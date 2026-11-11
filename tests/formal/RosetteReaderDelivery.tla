------------------------- MODULE RosetteReaderDelivery -------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
(***************************************************************************)
(* Reliable StatefulReaders of one participant, each matched to its own   *)
(* reliable writer, over a network that loses and reorders. It checks how *)
(* each reader makes its writer's changes available to the application:  *)
(* in the order written (DDSI-RTPS 2.5 §8.4.10.4, DDS 1.4 §2.2.3.17       *)
(* DESTINATION_ORDER for one writer), within shared bounded resources,   *)
(* without acknowledging a change the application never receives, and   *)
(* without one reader stalling another.                                    *)
(*                                                                         *)
(* Each writer (§8.4.9) writes changes 1..N, keeps every change not yet   *)
(* acknowledged cumulatively, announces them with HEARTBEATs, and resends *)
(* what an ACKNACK names. Its reader answers each HEARTBEAT with an       *)
(* ACKNACK whose base is its first change not yet received.               *)
(*                                                                         *)
(* Two shared resources bound the readers:                                 *)
(*  - Hold slots. A change that arrives ahead of a missing one waits in a *)
(*    slot of the participant's hold pool (Slots in all, at most Cap per  *)
(*    reader) until the gap below it closes. The change that closes a gap *)
(*    is delivered at once and needs no slot.                              *)
(*  - Buffer blocks. Every held change pins a block of the participant's  *)
(*    pool, and an arriving change needs a block before it can be         *)
(*    delivered or held. So does an ACKNACK: a reader whose held changes  *)
(*    take every block cannot answer a HEARTBEAT. Blocks counts the       *)
(*    blocks held changes and ACKNACKs share.                             *)
(*                                                                         *)
(* Rosette's choices, as constants:                                        *)
(*  - DeliverRule = "on_arrival" delivers each new change at once;        *)
(*    "in_order" holds a change until every earlier one is delivered or  *)
(*    given up.                                                            *)
(*  - AdmitRule decides which changes may be held. "window" holds only    *)
(*    the Cap changes after the next expected one, the ones the reader's  *)
(*    slots can always cover; "any" holds whatever finds a free slot. A   *)
(*    change that may not be held is dropped and requested again. The     *)
(*    ACKNACK requests every missing change either way: resends arrive    *)
(*    in order, and each closes the gap in turn without a slot.            *)
(*  - BlockRule decides what the change that closes a gap does when no    *)
(*    block is free: "drop_new" is dropped; "evict_own" takes the block of *)
(*    its own reader's highest held change; "evict_any" takes the block of *)
(*    the highest held change of any reader. An evicted change is missing *)
(*    again and is requested again.                                        *)
(*  - SilentWriter lets writer 1 stop for good mid-stream, as a writer    *)
(*    does before its lease expires; its reader's held changes then pin   *)
(*    their slots and blocks. The other readers must still finish.        *)
(*  - AckReserve holds a change only if a block stays free for the       *)
(*    reader's ACKNACK; without it, held changes can take every block and *)
(*    leave the reader unable to request the change that releases them.   *)
(*  - Depth = 0 is KEEP_ALL. Depth = k > 0 is KEEP_LAST k: once k changes *)
(*    are held behind a gap, the application could never see the missing *)
(*    changes below them, so the reader gives up the gap.                 *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    Readers,      \* Readers in the participant, each with its own writer.
    N,            \* Changes each writer writes.
    Cap,          \* Hold slots one reader may occupy.
    Slots,        \* Hold slots in the participant's pool.
    Blocks,       \* Buffer blocks held changes may pin, over all readers.
    Depth,        \* 0 = KEEP_ALL; k > 0 = KEEP_LAST k.
    Budget,       \* Messages the network may lose in a behavior.
    DeliverRule,  \* "on_arrival" or "in_order".
    AdmitRule,    \* "window" or "any".
    BlockRule,    \* "drop_new", "evict_own" or "evict_any".
    SilentWriter, \* TRUE lets writer 1 stop for good.
    AckReserve    \* TRUE: a change is held only if a block stays free for an ACKNACK.

ASSUME /\ Readers \in Nat \ {0} /\ N \in Nat \ {0}
       /\ Cap \in Nat \ {0} /\ Slots \in Nat \ {0} /\ Blocks \in Nat \ {0}
       /\ Depth \in Nat /\ Budget \in Nat
       /\ DeliverRule \in {"on_arrival", "in_order"}
       /\ AdmitRule \in {"window", "any"}
       /\ BlockRule \in {"drop_new", "evict_own", "evict_any"}
       /\ SilentWriter \in BOOLEAN /\ AckReserve \in BOOLEAN

R == 1..Readers
SN == 1..N

VARIABLES
    written,      \* written[r]: highest change writer r has written.
    acked,        \* acked[r]: highest change writer r has acknowledged cumulatively.
    alive,        \* alive[r]: FALSE once writer r has stopped.
    net,          \* Messages in flight (a set: duplicates merge, order is free).
    known,        \* known[r]: highest change reader r knows its writer wrote.
    held,         \* held[r]: changes reader r holds behind a gap.
    dlv,          \* dlv[r]: changes reader r delivered to the application.
    skipped,      \* skipped[r]: changes reader r gave up (KEEP_LAST only).
    lastDlv,      \* lastDlv[r]: the last change reader r delivered.
    disordered,   \* TRUE once a reader delivered a change after a later one.
    budget

vars == <<written, acked, alive, net, known, held, dlv, skipped, lastDlv, disordered, budget>>

Data(r, s)         == [t |-> "DATA", r |-> r, sn |-> s]
Heartbeat(r, f, l) == [t |-> "HB", r |-> r, first |-> f, last |-> l]
AckNack(r, b, m)   == [t |-> "ACK", r |-> r, base |-> b, missing |-> m]

Max(S) == CHOOSE x \in S : \A y \in S : y <= x
Min(S) == CHOOSE x \in S : \A y \in S : x <= y

Done(r) == dlv[r] \cup skipped[r]
Received(r) == Done(r) \cup held[r]

\* The next change in order for reader r: the first not delivered or given up.
NextOf(r) == IF SN \ Done(r) = {} THEN N + 1 ELSE Min(SN \ Done(r))

\* Every held change of every reader, as <<reader, change>> pairs.
HeldPairs(h) == UNION {{<<q, x>> : x \in h[q]} : q \in R}

TypeOK ==
    /\ written \in [R -> 0..N] /\ acked \in [R -> 0..N] /\ known \in [R -> 0..N]
    /\ alive \in [R -> BOOLEAN]
    /\ held \in [R -> SUBSET SN] /\ dlv \in [R -> SUBSET SN] /\ skipped \in [R -> SUBSET SN]
    /\ lastDlv \in [R -> 0..N] /\ disordered \in BOOLEAN /\ budget \in 0..Budget

Init ==
    /\ written = [r \in R |-> 0] /\ acked = [r \in R |-> 0] /\ alive = [r \in R |-> TRUE]
    /\ net = {} /\ known = [r \in R |-> 0]
    /\ held = [r \in R |-> {}] /\ dlv = [r \in R |-> {}] /\ skipped = [r \in R |-> {}]
    /\ lastDlv = [r \in R |-> 0] /\ disordered = FALSE /\ budget = Budget

-----------------------------------------------------------------------------
(* Delivery *)

\* Delivers the run of held changes that starts at the next change in
\* order. Returns <<delivered, held, last>>.
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

\* Commits reader r's delivery result <<d, hr, last>> and given-up set over
\* `h`, the hold pool after any eviction from another reader.
Commit(r, sk, res, h) ==
    /\ skipped' = [skipped EXCEPT ![r] = sk]
    /\ dlv' = [dlv EXCEPT ![r] = res[1]]
    /\ held' = [h EXCEPT ![r] = res[2]]
    /\ lastDlv' = [lastDlv EXCEPT ![r] = res[3]]
    /\ disordered' = (disordered \/ \E x \in res[1] \ dlv[r] : x < lastDlv[r])

Deliver(r, s, h) == Commit(r, skipped[r], Release(dlv[r] \cup {s}, h[r], skipped[r], s), h)

Hold(r, s) ==
    LET g == GiveUp(dlv[r], held[r] \cup {s}, skipped[r], lastDlv[r])
    IN Commit(r, g[1], g[2], held)

Evict(h, q) == [h EXCEPT ![q] = @ \ {Max(@)}]

Drop == UNCHANGED <<held, dlv, skipped, lastDlv, disordered>>

BlockFree == Cardinality(HeldPairs(held)) < Blocks

\* A block a change may take to be held: under AckReserve, one that leaves
\* another free for the reader's ACKNACK.
HoldBlockFree == Cardinality(HeldPairs(held)) + (IF AckReserve THEN 1 ELSE 0) < Blocks

\* A slot reader r may take for change s.
SlotFor(r, s) ==
    /\ Cardinality(held[r]) < Cap
    /\ Cardinality(HeldPairs(held)) < Slots
    /\ (AdmitRule = "any" \/ s <= NextOf(r) + Cap)

\* A new change s reaches reader r.
Admit(r, s) ==
    IF DeliverRule = "on_arrival" \/ s = NextOf(r)
    THEN IF BlockFree THEN Deliver(r, s, held)
         ELSE IF BlockRule = "evict_own" /\ held[r] # {} THEN Deliver(r, s, Evict(held, r))
         ELSE IF BlockRule = "evict_any"
              THEN \E q \in {x \in R : held[x] # {}} : Deliver(r, s, Evict(held, q))
         ELSE Drop
    ELSE IF SlotFor(r, s) /\ HoldBlockFree THEN Hold(r, s) ELSE Drop

RecvData(r, s) ==
    /\ Data(r, s) \in net
    /\ net' = net \ {Data(r, s)}
    /\ known' = [known EXCEPT ![r] = IF s > @ THEN s ELSE @]
    /\ IF s \in Received(r) THEN Drop ELSE Admit(r, s)
    /\ UNCHANGED <<written, acked, alive, budget>>

\* §8.4.12.2: a HEARTBEAT is answered with an ACKNACK, when a block is free
\* to build it in. A change below the HEARTBEAT's first is no longer
\* available from the writer.
RecvHeartbeat(r, f, l) ==
    /\ Heartbeat(r, f, l) \in net
    /\ LET k == IF l > known[r] THEN l ELSE known[r]
           gone == IF DeliverRule = "in_order"
                   THEN {s \in SN : s < f /\ s \notin Received(r)} ELSE {}
           sk == skipped[r] \cup gone
           res == Release(dlv[r], held[r], sk, lastDlv[r])
           have == dlv[r] \cup sk \cup held[r]
           base == IF (1..k) \ have = {} THEN k + 1 ELSE Min((1..k) \ have)
       IN /\ known' = [known EXCEPT ![r] = k]
          /\ Commit(r, sk, res, held)
          /\ net' = (net \ {Heartbeat(r, f, l)})
                    \cup (IF BlockFree THEN {AckNack(r, base, {s \in base..k : s \notin have})}
                          ELSE {})
    /\ UNCHANGED <<written, acked, alive, budget>>

-----------------------------------------------------------------------------
(* Writers and network *)

Write(r) ==
    /\ alive[r] /\ written[r] < N
    /\ written' = [written EXCEPT ![r] = @ + 1]
    /\ net' = net \cup {Data(r, written[r] + 1)}
    /\ UNCHANGED <<acked, alive, known, held, dlv, skipped, lastDlv, disordered, budget>>

\* §8.4.9.2.7: the periodic HEARTBEAT while changes are unacknowledged.
SendHeartbeat(r) ==
    /\ alive[r] /\ acked[r] < written[r]
    /\ Heartbeat(r, acked[r] + 1, written[r]) \notin net
    /\ net' = net \cup {Heartbeat(r, acked[r] + 1, written[r])}
    /\ UNCHANGED <<written, acked, alive, known, held, dlv, skipped, lastDlv, disordered, budget>>

\* §8.4.9.2.8: an ACKNACK acknowledges below its base and requests its set.
RecvAckNack(m) ==
    /\ m \in net /\ m.t = "ACK" /\ alive[m.r]
    /\ LET a == IF m.base - 1 > acked[m.r] THEN m.base - 1 ELSE acked[m.r]
       IN /\ acked' = [acked EXCEPT ![m.r] = a]
          /\ net' = (net \ {m}) \cup {Data(m.r, s) : s \in {x \in m.missing : x > a}}
    /\ UNCHANGED <<written, alive, known, held, dlv, skipped, lastDlv, disordered, budget>>

\* Writer 1 stops for good: no more changes, HEARTBEATs or resends.
Silence ==
    /\ SilentWriter /\ alive[1]
    /\ alive' = [alive EXCEPT ![1] = FALSE]
    /\ UNCHANGED <<written, acked, net, known, held, dlv, skipped, lastDlv, disordered, budget>>

Lose(m) ==
    /\ budget > 0 /\ m \in net
    /\ net' = net \ {m}
    /\ budget' = budget - 1
    /\ UNCHANGED <<written, acked, alive, known, held, dlv, skipped, lastDlv, disordered>>

Next_ ==
    \/ \E r \in R : Write(r) \/ SendHeartbeat(r)
    \/ \E r \in R, s \in SN : RecvData(r, s)
    \/ \E r \in R, f \in 1..N, l \in 1..N : RecvHeartbeat(r, f, l)
    \/ \E m \in net : RecvAckNack(m) \/ Lose(m)
    \/ Silence

Spec == Init /\ [][Next_]_vars

FairSpec ==
    /\ Spec
    /\ \A r \in R : WF_vars(Write(r)) /\ WF_vars(SendHeartbeat(r))
    /\ \A r \in R, s \in SN : WF_vars(RecvData(r, s))
    /\ \A r \in R, f \in 1..N, l \in 1..N : WF_vars(RecvHeartbeat(r, f, l))
    /\ \A r \in R : WF_vars(\E m \in net : m.r = r /\ RecvAckNack(m))

-----------------------------------------------------------------------------
(* Properties *)

\* Changes from one writer reach the application in the order written.
InOrder == ~disordered

\* No writer counts as acknowledged a change the application has neither
\* received nor lost to KEEP_LAST: an acknowledged change is never resent,
\* so one dropped after its acknowledgement is lost silently.
NoAckPastUndelivered == \A r \in R : acked[r] <= NextOf(r) - 1

\* Held changes stay within each reader's cap, the pool, and the blocks.
WithinCap ==
    /\ \A r \in R : Cardinality(held[r]) <= Cap
    /\ Cardinality(HeldPairs(held)) <= Slots
    /\ Cardinality(HeldPairs(held)) <= Blocks

\* Under the window rule a reader holds only the Cap changes after its next.
HoldsInWindow == AdmitRule = "window" => \A r \in R : \A s \in held[r] : s <= NextOf(r) + Cap

\* Only a KEEP_LAST reader gives a change up.
SkipOnlyKeepLast == \A r \in R : skipped[r] # {} => Depth > 0

-----------------------------------------------------------------------------
(* Witnesses *)

\* Each claim below is false in any configuration that exercises the rule
\* it names, so TLC must report it violated. A configuration in which one
\* holds never reaches that rule, and its other results say nothing about
\* it. The action witnesses are checked step by step and add no state.

\* A reader never holds a change in every slot its cap allows.
NeverHoldsCap == \A r \in R : Cardinality(held[r]) < Cap

\* The participant's hold pool is never full.
NeverPoolFull == Cardinality(HeldPairs(held)) < Slots

\* Reader r receives a new change s: a step that takes DATA s from the
\* network without losing it.
Arrives(r, s) ==
    /\ Data(r, s) \in net /\ Data(r, s) \notin net' /\ budget' = budget
    /\ s \notin Received(r)

\* A change past a reader's window is refused.
RefusedPastWindow == \E r \in R, s \in SN : Arrives(r, s) /\ s > NextOf(r) + Cap
NeverRefusedPastWindow == [][~RefusedPastWindow]_vars

\* A change inside a reader's window, with room under its cap, is refused
\* because other readers fill the shared pool.
RefusedForPool ==
    \E r \in R, s \in SN :
        /\ Arrives(r, s) /\ NextOf(r) < s /\ s <= NextOf(r) + Cap
        /\ Cardinality(held[r]) < Cap /\ Cardinality(HeldPairs(held)) = Slots
NeverRefusedForPool == [][~RefusedForPool]_vars

\* A change that closes one reader's gap takes another reader's block.
EvictedAcrossReaders ==
    \E r, q \in R :
        /\ r # q /\ dlv'[r] # dlv[r]
        /\ \E x \in held[q] : x \notin held'[q] /\ x \notin dlv'[q]
NeverEvictedAcrossReaders == [][~EvictedAcrossReaders]_vars

-----------------------------------------------------------------------------
(* Liveness *)

\* Every reader whose writer keeps going eventually has every change
\* delivered or, under KEEP_LAST, given up.
LiveReaders == IF SilentWriter THEN R \ {1} ELSE R
EveryChangeDone == \A r \in LiveReaders : <>(written[r] = N /\ Done(r) = SN)

=============================================================================
