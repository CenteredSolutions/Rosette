------------------------- MODULE RosetteReaderDelivery -------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
(***************************************************************************)
(* One reliable writer, one reliable StatefulReader, and a network that   *)
(* loses and reorders. It checks how the reader makes a writer's changes  *)
(* available to the application: in the order written (DDSI-RTPS 2.5      *)
(* §8.4.10.4, DDS 1.4 §2.2.3.17 DESTINATION_ORDER for one writer), with   *)
(* bounded buffers, without acknowledging a change the application never *)
(* receives, and without getting stuck.                                    *)
(*                                                                         *)
(* The writer (§8.4.9) writes changes 1..N, keeps every change not yet    *)
(* acknowledged cumulatively, announces them with HEARTBEATs, and resends *)
(* what an ACKNACK names. The reader answers each HEARTBEAT with an       *)
(* ACKNACK whose base is its first change not yet received and whose set  *)
(* names the changes it is missing up to the HEARTBEAT's last.            *)
(*                                                                         *)
(* An arriving change needs a buffer before it can be delivered or held   *)
(* (StatefulReader::on_data() copies it into a pool block). A change that *)
(* arrives ahead of a missing one is held in a block until the gap below  *)
(* it closes. Cap is the number of blocks this reader may occupy.         *)
(*                                                                         *)
(* Rosette's choices, as constants:                                        *)
(*  - DeliverRule = "on_arrival" delivers each new change at once;        *)
(*    "in_order" holds a change until every earlier one is delivered or  *)
(*    given up.                                                            *)
(*  - FullRule decides what happens when every block is in use and a      *)
(*    change arrives: "drop_new" drops the arriving change; "evict_highest"*)
(*    frees the block of the highest held change when the arriving one is *)
(*    lower (the evicted change is missing again and is NACKed again), and *)
(*    drops the arriving one otherwise.                                    *)
(*  - Depth = 0 is KEEP_ALL. Depth = k > 0 is KEEP_LAST k: once k changes *)
(*    are held behind a gap, the application could never see the missing *)
(*    changes below them (a history of depth k keeps only the newest k),  *)
(*    so the reader gives up the gap and delivers what it holds.           *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    N,            \* Changes the writer writes.
    Cap,          \* Buffer blocks the reader may occupy.
    Depth,        \* 0 = KEEP_ALL; k > 0 = KEEP_LAST k.
    Budget,       \* Messages the network may lose in a behavior.
    DeliverRule,  \* "on_arrival" or "in_order".
    FullRule      \* "drop_new" or "evict_highest".

ASSUME /\ N \in Nat \ {0} /\ Cap \in Nat \ {0} /\ Depth \in Nat /\ Budget \in Nat
       /\ DeliverRule \in {"on_arrival", "in_order"}
       /\ FullRule \in {"drop_new", "evict_highest"}

SN == 1..N

VARIABLES
    written,      \* Highest change the writer has written.
    acked,        \* Writer: highest change acknowledged cumulatively.
    net,          \* Messages in flight (a set: duplicates merge, order is free).
    known,        \* Reader: highest change it knows the writer wrote.
    held,         \* Reader: changes held in a block behind a gap.
    dlv,          \* Reader: changes delivered to the application.
    skipped,      \* Reader: changes given up (KEEP_LAST only).
    lastDlv,      \* Reader: the last change delivered.
    disordered,   \* TRUE once a change was delivered after a later one.
    budget

vars == <<written, acked, net, known, held, dlv, skipped, lastDlv, disordered, budget>>

Data(s)       == [t |-> "DATA", sn |-> s]
Heartbeat(f, l) == [t |-> "HB", first |-> f, last |-> l]
AckNack(b, m) == [t |-> "ACK", base |-> b, missing |-> m]

Max(S) == CHOOSE x \in S : \A y \in S : y <= x
Min(S) == CHOOSE x \in S : \A y \in S : x <= y

\* Changes the reader has: delivered, given up, or held.
Done == dlv \cup skipped
Received == Done \cup held

\* The next change in order: the first not yet delivered or given up.
Next == IF SN \ Done = {} THEN N + 1 ELSE Min(SN \ Done)

\* The highest change below which every change is delivered or given up.
DonePrefix == Next - 1

TypeOK ==
    /\ written \in 0..N /\ acked \in 0..N /\ known \in 0..N
    /\ held \subseteq SN /\ dlv \subseteq SN /\ skipped \subseteq SN
    /\ lastDlv \in 0..N /\ disordered \in BOOLEAN /\ budget \in 0..Budget

Init ==
    /\ written = 0 /\ acked = 0 /\ net = {} /\ known = 0
    /\ held = {} /\ dlv = {} /\ skipped = {} /\ lastDlv = 0
    /\ disordered = FALSE /\ budget = Budget

-----------------------------------------------------------------------------
(* Delivery *)

\* Delivers the run of held changes that now starts at Next, from `d`
\* (delivered) and `h` (held). Returns <<delivered, held, last>>.
RECURSIVE Release(_, _, _, _)
Release(d, h, sk, last) ==
    LET nx == IF SN \ (d \cup sk) = {} THEN N + 1 ELSE Min(SN \ (d \cup sk))
    IN IF nx \in h THEN Release(d \cup {nx}, h \ {nx}, sk, nx) ELSE <<d, h, last>>

\* KEEP_LAST k: with k changes held behind a gap, give up every missing
\* change below the lowest held one and deliver the run that follows.
GiveUp(d, h, sk, last) ==
    IF Depth > 0 /\ Cardinality(h) >= Depth
    THEN LET gone == {s \in SN : s < Min(h) /\ s \notin d \cup sk}
         IN <<sk \cup gone, Release(d, h, sk \cup gone, last)>>
    ELSE <<sk, <<d, h, last>>>>

\* Commits a delivery result <<d, h, last>> and the given-up set.
Commit(sk, r) ==
    /\ skipped' = sk
    /\ dlv' = r[1] /\ held' = r[2] /\ lastDlv' = r[3]
    /\ disordered' = (disordered \/ \E s \in r[1] \ dlv : s < lastDlv)

\* A change arrives with a block free for it.
Accept(s, h) ==
    IF DeliverRule = "on_arrival" \/ s = Next
    THEN LET r == Release(dlv \cup {s}, h, skipped, s)
         IN Commit(skipped, r)
    ELSE LET g == GiveUp(dlv, h \cup {s}, skipped, lastDlv)
         IN Commit(g[1], g[2])

RecvData(s) ==
    /\ Data(s) \in net
    /\ net' = net \ {Data(s)}
    /\ known' = IF s > known THEN s ELSE known
    /\ IF s \in Received
       THEN UNCHANGED <<held, dlv, skipped, lastDlv, disordered>>
       ELSE IF Cardinality(held) < Cap
       THEN Accept(s, held)
       ELSE IF FullRule = "evict_highest" /\ held # {} /\ s < Max(held)
       THEN Accept(s, held \ {Max(held)})
       ELSE UNCHANGED <<held, dlv, skipped, lastDlv, disordered>>
    /\ UNCHANGED <<written, acked, budget>>

\* §8.4.12.2: a HEARTBEAT is answered with an ACKNACK. A change below
\* the HEARTBEAT's first is no longer available from the writer.
RecvHeartbeat(f, l) ==
    /\ Heartbeat(f, l) \in net
    /\ LET k == IF l > known THEN l ELSE known
           gone == IF DeliverRule = "in_order"
                   THEN {s \in SN : s < f /\ s \notin Received} ELSE {}
           r == Release(dlv, held, skipped \cup gone, lastDlv)
           sk == skipped \cup gone
           have == dlv \cup sk \cup held
           base == IF (1..k) \ have = {} THEN k + 1 ELSE Min((1..k) \ have)
       IN /\ known' = k
          /\ Commit(sk, r)
          /\ net' = (net \ {Heartbeat(f, l)})
                    \cup {AckNack(base, {s \in base..k : s \notin have})}
    /\ UNCHANGED <<written, acked, budget>>

-----------------------------------------------------------------------------
(* Writer and network *)

Write ==
    /\ written < N
    /\ written' = written + 1
    /\ net' = net \cup {Data(written + 1)}
    /\ UNCHANGED <<acked, known, held, dlv, skipped, lastDlv, disordered, budget>>

\* §8.4.9.2.7: the periodic HEARTBEAT while changes are unacknowledged.
SendHeartbeat ==
    /\ acked < written
    /\ Heartbeat(acked + 1, written) \notin net
    /\ net' = net \cup {Heartbeat(acked + 1, written)}
    /\ UNCHANGED <<written, acked, known, held, dlv, skipped, lastDlv, disordered, budget>>

\* §8.4.9.2.8: an ACKNACK acknowledges below its base and requests its set.
RecvAckNack(m) ==
    /\ m \in net /\ m.t = "ACK"
    /\ LET a == IF m.base - 1 > acked THEN m.base - 1 ELSE acked
       IN /\ acked' = a
          /\ net' = (net \ {m}) \cup {Data(s) : s \in {x \in m.missing : x > a}}
    /\ UNCHANGED <<written, known, held, dlv, skipped, lastDlv, disordered, budget>>

Lose(m) ==
    /\ budget > 0 /\ m \in net
    /\ net' = net \ {m}
    /\ budget' = budget - 1
    /\ UNCHANGED <<written, acked, known, held, dlv, skipped, lastDlv, disordered>>

Next_ ==
    \/ Write \/ SendHeartbeat
    \/ \E s \in SN : RecvData(s)
    \/ \E f \in 1..N, l \in 1..N : RecvHeartbeat(f, l)
    \/ \E m \in net : RecvAckNack(m) \/ Lose(m)

Spec == Init /\ [][Next_]_vars

FairSpec ==
    /\ Spec
    /\ WF_vars(Write) /\ WF_vars(SendHeartbeat)
    /\ \A s \in SN : WF_vars(RecvData(s))
    /\ \A f \in 1..N, l \in 1..N : WF_vars(RecvHeartbeat(f, l))
    /\ WF_vars(\E m \in net : RecvAckNack(m))

-----------------------------------------------------------------------------
(* Properties *)

\* Changes from one writer reach the application in the order written.
InOrder == ~disordered

\* The writer never counts as acknowledged a change the application has
\* neither received nor lost to KEEP_LAST: an acknowledged change is never
\* resent, so one dropped after its acknowledgement is lost silently.
NoAckPastUndelivered == acked <= DonePrefix

\* The reader occupies no more blocks than it may.
WithinCap == Cardinality(held) <= Cap

\* Only a KEEP_LAST reader gives a change up.
SkipOnlyKeepLast == skipped # {} => Depth > 0

\* Every change is eventually delivered or, under KEEP_LAST, given up.
EveryChangeDone == <>(written = N /\ Done = SN)

=============================================================================
