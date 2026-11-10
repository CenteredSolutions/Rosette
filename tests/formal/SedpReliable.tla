---------------------------- MODULE SedpReliable ----------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
(***************************************************************************)
(* What DDSI-RTPS 2.5 requires and permits of one reliable SEDP channel:   *)
(* a remote participant's publications announcer, the network between     *)
(* it and the local participant, and the bookkeeping a reliable reader's   *)
(* WriterProxy performs.                                                    *)
(*                                                                         *)
(* This module holds no Rosette choice. Where the specification leaves a  *)
(* choice open, the action is nondeterministic, so a module that extends   *)
(* this one is checked against every compliant peer, not against one       *)
(* vendor's behavior:                                                       *)
(*                                                                         *)
(*  - The announcer may send any DATA it holds, a GAP for any run of       *)
(*    changes it no longer holds, and a HEARTBEAT describing its history,  *)
(*    at any time (§8.4.7, §8.4.9). ACKNACKs are therefore not modelled:   *)
(*    whatever a reader requests, the writer was already free to send.     *)
(*  - The network loses, duplicates and reorders messages (§8.2: RTPS      *)
(*    assumes a best-effort, unordered transport such as UDP).             *)
(*  - Withdrawing an endpoint writes a disposal and makes its earlier      *)
(*    announcement irrelevant (§8.5.4.1, §9.6.3.3). A disposal may itself  *)
(*    leave the history once the matched readers hold it; a late joiner    *)
(*    then receives a GAP for both (rosette::rtps::SedpWriter and Fast DDS *)
(*    behave so). The extending module decides what "the matched readers  *)
(*    hold it" means, since it owns the reader.                            *)
(*                                                                         *)
(* One announcer stands for both: the subscriptions announcer follows the  *)
(* same rules on its own channel. An endpoint GUID is never reused once    *)
(* withdrawn (§8.2.4.3: entity IDs are unique within a participant).      *)
(*                                                                         *)
(* Read with: docs/design/two_tier_peer_table.md and tests/formal/README.md.     *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    Endpoints,   \* GUIDs of the peer's endpoints (model values).
    MaxSN,       \* Bound on the announcer's sequence numbers.
    NetCap       \* Bound on the messages in flight at once.

ASSUME MaxSN \in Nat /\ NetCap \in Nat \ {0}

SNs == 1..MaxSN
Max(a, b) == IF a >= b THEN a ELSE b

(***************************************************************************)
(* The announcer's history (§8.5.4.1). `written` is what each sequence    *)
(* number was written as, which never changes; `irrelevant` holds the     *)
(* numbers the writer no longer holds and answers with a GAP.             *)
(***************************************************************************)
Alive(e)   == [kind |-> "alive", ep |-> e]
Dispose(e) == [kind |-> "dispose", ep |-> e]
NoChange   == [kind |-> "none"]
Changes    == {Alive(e) : e \in Endpoints} \cup {Dispose(e) : e \in Endpoints}

DataMsg(sn)     == [type |-> "DATA", sn |-> sn]
GapMsg(lo, hi)  == [type |-> "GAP", lo |-> lo, hi |-> hi]   \* [lo, hi] irrelevant
HbMsg(first, l) == [type |-> "HB", first |-> first, last |-> l]
Messages ==
    {DataMsg(s) : s \in SNs}
    \cup {GapMsg(lo, hi) : lo \in SNs, hi \in SNs}
    \cup {HbMsg(f, l) : f \in 1..(MaxSN + 1), l \in 0..MaxSN}

VARIABLES
    written,     \* [SNs -> Changes \cup {NoChange}]
    wsn,         \* Highest sequence number written, 0..MaxSN.
    irrelevant,  \* SUBSET 1..wsn: changes the writer no longer holds.
    withdrawn,   \* Endpoints withdrawn, ever; never announced again.
    net          \* Messages in flight.

peerVars == <<written, wsn, irrelevant, withdrawn>>

Announced == {written[s].ep : s \in 1..wsn}
LiveSNs(e) == {s \in 1..wsn : written[s] = Alive(e) /\ s \notin irrelevant}

\* The endpoints the peer has right now: the truth a listing converges to.
PeerLive == {e \in Endpoints : LiveSNs(e) # {}}

\* §8.3.7.5: firstSN is the lowest change the writer holds; with an empty
\* history it is lastSN + 1.
Held == {s \in 1..wsn : s \notin irrelevant}
FirstHeld == IF Held = {} THEN wsn + 1 ELSE CHOOSE s \in Held : \A t \in Held : s <= t

PeerTypeOK ==
    /\ written \in [SNs -> Changes \cup {NoChange}]
    /\ wsn \in 0..MaxSN
    /\ irrelevant \subseteq 1..wsn
    /\ withdrawn \subseteq Endpoints
    /\ net \subseteq Messages
    /\ Cardinality(net) <= NetCap

PeerInit ==
    /\ written = [s \in SNs |-> NoChange]
    /\ wsn = 0
    /\ irrelevant = {}
    /\ withdrawn = {}
    /\ net = {}

Write(change, superseded) ==
    /\ wsn < MaxSN
    /\ written' = [written EXCEPT ![wsn + 1] = change]
    /\ wsn' = wsn + 1
    /\ irrelevant' = irrelevant \cup superseded

\* The peer creates an endpoint.
Announce(e) ==
    /\ e \notin Announced
    /\ Write(Alive(e), {})
    /\ UNCHANGED <<withdrawn, net>>

\* The peer announces a live endpoint again, e.g. with new locators or QoS;
\* the earlier announcement becomes irrelevant.
Reannounce(e) ==
    /\ e \in PeerLive
    /\ Write(Alive(e), LiveSNs(e))
    /\ UNCHANGED <<withdrawn, net>>

\* The peer deletes an endpoint (§9.6.3.3 disposal).
Withdraw(e) ==
    /\ e \in PeerLive
    /\ Write(Dispose(e), LiveSNs(e))
    /\ withdrawn' = withdrawn \cup {e}
    /\ UNCHANGED net

\* A disposal leaves the history once every matched reader holds it.
PurgeDisposal(s, heldByMatchedReaders) ==
    /\ s \in Held
    /\ written[s].kind = "dispose"
    /\ heldByMatchedReaders
    /\ irrelevant' = irrelevant \cup {s}
    /\ UNCHANGED <<written, wsn, withdrawn, net>>

Send(m) ==
    /\ m \notin net
    /\ Cardinality(net) < NetCap
    /\ net' = net \cup {m}
    /\ UNCHANGED peerVars

\* What the announcer may send right now.
Sendable(m) ==
    CASE m.type = "DATA" -> m.sn \in Held
      [] m.type = "GAP"  -> m.lo <= m.hi /\ m.hi <= wsn /\ m.lo..m.hi \subseteq irrelevant
      [] m.type = "HB"   -> m = HbMsg(FirstHeld, wsn)

SendData(s) == Sendable(DataMsg(s)) /\ Send(DataMsg(s))
SendGap(lo, hi) == Sendable(GapMsg(lo, hi)) /\ Send(GapMsg(lo, hi))
SendHeartbeat == Send(HbMsg(FirstHeld, wsn))

Lose(m) == net' = net \ {m} /\ UNCHANGED peerVars

\* Every step the peer and the network may take on their own. The extending
\* module adds PurgeDisposal with its own notion of a held disposal.
PeerNext ==
    \/ \E e \in Endpoints : Announce(e) \/ Reannounce(e) \/ Withdraw(e)
    \/ \E s \in SNs : SendData(s)
    \/ \E lo, hi \in SNs : SendGap(lo, hi)
    \/ SendHeartbeat
    \/ \E m \in net : Lose(m)

(***************************************************************************)
(* A reliable reader's WriterProxy (§8.4.10.4). Each sequence number is   *)
(* unknown, received (DATA), irrelevant (GAP) or lost (below a            *)
(* HEARTBEAT's firstSN); the last three are resolved. `lastAvail` is the  *)
(* highest number the writer is known to hold: an unresolved number at or *)
(* below it is MISSING.                                                    *)
(***************************************************************************)
ProxyStates == {"unknown", "received", "irrelevant", "lost"}
FreshProxy == [st |-> [s \in SNs |-> "unknown"], lastAvail |-> 0, hbSeen |-> FALSE]
Proxies == [st : [SNs -> ProxyStates], lastAvail : 0..MaxSN, hbSeen : BOOLEAN]

Resolved(p, s) == p.st[s] # "unknown"

\* §8.4.10.4 available_changes_max: every change up to it is resolved.
AvailMax(p) ==
    CHOOSE k \in 0..MaxSN :
        /\ \A s \in 1..k : Resolved(p, s)
        /\ (k = MaxSN \/ ~Resolved(p, k + 1))

HasMissing(p) == \E s \in 1..p.lastAvail : ~Resolved(p, s)

\* §8.4.10.4 received_change_set and missing_changes_update.
ReceivedChangeSet(p, s) ==
    [p EXCEPT !.st[s] = "received", !.lastAvail = Max(@, s)]

\* §8.4.10.4 irrelevant_change_set: a received change stays received.
IrrelevantChangeSet(p, lo, hi) ==
    [p EXCEPT !.st = [s \in SNs |-> IF s \in lo..hi /\ @[s] = "unknown" THEN "irrelevant" ELSE @[s]],
              !.lastAvail = Max(@, hi)]

\* §8.4.10.4 lost_changes_update and missing_changes_update, from a HEARTBEAT.
HeartbeatUpdate(p, first, last) ==
    [p EXCEPT !.st = [s \in SNs |-> IF s < first /\ @[s] = "unknown" THEN "lost" ELSE @[s]],
              !.lastAvail = Max(@, last),
              !.hbSeen = TRUE]

=============================================================================
