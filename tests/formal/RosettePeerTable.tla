-------------------------- MODULE RosettePeerTable --------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
(***************************************************************************)
(* The two-tier peer table as a whole: several remote participants, fewer *)
(* active slots, and the rules that decide who holds one                  *)
(* (src/rtps/participant_peer_tiers.cpp, docs/design/two_tier_peer_table.md).*)
(*                                                                         *)
(* RosettePeerTiers checks one peer's SEDP exchange message by message.   *)
(* Here that exchange is one step, Exchange(p), which an active peer that  *)
(* can be reached eventually takes: it processes what the peer announced,  *)
(* has this side's announcements acknowledged, and learns whether the peer *)
(* shares a match with a local endpoint. What this module adds is the     *)
(* competition for slots:                                                  *)
(*                                                                         *)
(*  - Admission (admit_peer()): on a new peer's SPDP announcement, again on *)
(*    each announcement while it is pending, on a parked peer's SEDP       *)
(*    traffic above its parked number, and for an owed peer from tick().  *)
(*    With every slot taken, the settled peer whose last SEDP DATA is      *)
(*    oldest is parked to make room (park_settled_peer()); with none       *)
(*    settled, admission fails and is retried by the next trigger.         *)
(*  - Settled (peer_is_settled()): no match with a local endpoint, this    *)
(*    side's announcements acknowledged, and the settle timer run out. The *)
(*    reader's catch-up is not required here: a peer's new announcement   *)
(*    may still be in flight, which is why parked traffic promotes.       *)
(*  - Owed (owe_parked_peers()): every local endpoint created or deleted   *)
(*    marks each parked peer owed; serve_owed_peer() admits one owed peer  *)
(*    per tick. ServeOrder is Rosette's choice of which: "table" takes the *)
(*    first in table order; "oldest" the one whose last SEDP DATA or       *)
(*    activation is oldest, which every reactivation of another peer moves *)
(*    toward the front, so each owed peer is eventually first. "oldest" is *)
(*    modelled as any owed peer, with fairness for each.                   *)
(*  - Lease expiry or disposal removes a peer from either tier; it may come *)
(*    back later as a new participant.                                     *)
(*                                                                         *)
(* Peers in Unreachable announce by SPDP multicast, but their SEDP         *)
(* exchange never completes (unicast filtered, a NAT, a firewall).         *)
(*                                                                         *)
(* StallRule is Rosette's choice for such a peer. "none": it is never      *)
(* settled, so never parked, and holds its slot for as long as SPDP keeps  *)
(* it alive. "deadline": an active peer with no match whose exchange has   *)
(* not completed RTPS_PEER_EXCHANGE_TIMEOUT_MS after its last SEDP DATA is *)
(* stalled, and may be parked like a settled peer; it stays owed, so it is *)
(* tried again when a slot is free. The deadline is assumed to outlast any *)
(* exchange that can complete, as phase 1 assumes a datagram's delay is   *)
(* shorter than the settle time.                                           *)
(*                                                                         *)
(* The environment (joins, leaves, announcements, local endpoint changes)  *)
(* draws on a finite Budget, so every behavior goes quiet and liveness is  *)
(* checked on what the table does after that.                              *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets, Sequences

CONSTANTS
    N,            \* Remote participants, 1..N in table order.
    Slots,        \* RTPS_MAX_PEERS: active slots.
    Records,      \* RTPS_MAX_KNOWN_PEERS: known-peer records.
    Unreachable,  \* Peers whose SEDP exchange never completes.
    Budget,       \* Environment events in a behavior.
    StallRule,    \* "none" or "deadline".
    ServeOrder    \* "table" or "oldest".

Peers == 1..N
ASSUME Slots \in 1..N /\ Records \in Slots..N /\ Unreachable \subseteq Peers
ASSUME StallRule \in {"none", "deadline"} /\ ServeOrder \in {"table", "oldest"}

VARIABLES
    alive,     \* The participant exists and its lease is current.
    status,    \* "unknown", "pending", "active" or "parked".
    peerNews,  \* It announced something this side has not processed.
    ourNews,   \* This side announced something it has not acknowledged.
    owed,      \* Parked and owed this side's announcements.
    shares,    \* Truth: it has an endpoint matching a local endpoint.
    matched,   \* What the match table knows of `shares`, from the last exchange.
    settled,   \* peer_is_settled() holds.
    stalled,   \* peer_is_stalled() holds.
    lru,       \* Active peers, oldest SEDP DATA first.
    budget

vars == <<alive, status, peerNews, ourNews, owed, shares, matched, settled, stalled, lru, budget>>

Active == {p \in Peers : status[p] = "active"}
Known == {p \in Peers : status[p] # "unknown"}
Without(s, p) == SelectSeq(s, LAMBDA q : q # p)
SeqSet(s) == {s[i] : i \in 1..Len(s)}

TypeOK ==
    /\ alive \in [Peers -> BOOLEAN]
    /\ status \in [Peers -> {"unknown", "pending", "active", "parked"}]
    /\ peerNews \in [Peers -> BOOLEAN] /\ ourNews \in [Peers -> BOOLEAN]
    /\ owed \in [Peers -> BOOLEAN] /\ shares \in [Peers -> BOOLEAN]
    /\ matched \in [Peers -> BOOLEAN] /\ settled \in [Peers -> BOOLEAN]
    /\ stalled \in [Peers -> BOOLEAN]
    /\ budget \in 0..Budget

Init ==
    /\ alive = [p \in Peers |-> FALSE]
    /\ status = [p \in Peers |-> "unknown"]
    /\ peerNews = [p \in Peers |-> FALSE]
    /\ ourNews = [p \in Peers |-> FALSE]
    /\ owed = [p \in Peers |-> FALSE]
    /\ shares = [p \in Peers |-> FALSE]
    /\ matched = [p \in Peers |-> FALSE]
    /\ settled = [p \in Peers |-> FALSE]
    /\ stalled = [p \in Peers |-> FALSE]
    /\ lru = <<>>
    /\ budget = Budget

(***************************************************************************)
(* admit_peer(q), as one step: activate q in a free slot, or in the slot  *)
(* of the settled or stalled peer whose last SEDP DATA is oldest, parked  *)
(* for it; or leave everything as it is. A stalled peer is parked owed.   *)
(***************************************************************************)
Parkable(p) == settled[p] \/ stalled[p]
ParkableActive == {p \in Active : Parkable(p)}
OldestParkable == lru[CHOOSE i \in 1..Len(lru) :
                         /\ Parkable(lru[i])
                         /\ \A j \in 1..(i - 1) : ~Parkable(lru[j])]

\* activate_peer(): a fresh proxy replays the peer's announcements, and this
\* side's are sent to it again; the match table learns its endpoints anew.
Activated(q, st) == [st EXCEPT ![q] = "active"]

\* `st` is the status map admission starts from; it is kept when admission
\* fails.
Admit(q, st) ==
    IF st[q] = "active"
    THEN UNCHANGED <<status, peerNews, ourNews, owed, matched, settled, stalled, lru>>
    ELSE IF Cardinality(Active) < Slots \/ ParkableActive # {}
    THEN LET full == Cardinality(Active) >= Slots
             v == IF full THEN OldestParkable ELSE q
         IN /\ status' = Activated(q, IF full THEN [st EXCEPT ![v] = "parked"] ELSE st)
            /\ peerNews' = [peerNews EXCEPT ![q] = TRUE]
            /\ ourNews' = [ourNews EXCEPT ![q] = TRUE]
            /\ owed' = [owed EXCEPT ![v] = full /\ stalled[v], ![q] = FALSE]
            /\ matched' = [matched EXCEPT ![q] = FALSE, ![v] = FALSE]
            /\ settled' = [settled EXCEPT ![q] = FALSE, ![v] = FALSE]
            /\ stalled' = [stalled EXCEPT ![q] = FALSE, ![v] = FALSE]
            /\ lru' = Append(Without(lru, v), q)
    ELSE /\ status' = st
         /\ UNCHANGED <<peerNews, ourNews, owed, matched, settled, stalled, lru>>

(***************************************************************************)
(* The environment.                                                         *)
(***************************************************************************)
Spend == budget > 0 /\ budget' = budget - 1

\* A participant starts; its endpoints may or may not match local ones.
Join(p) ==
    /\ ~alive[p]
    /\ Spend
    /\ alive' = [alive EXCEPT ![p] = TRUE]
    /\ \E s \in BOOLEAN : shares' = [shares EXCEPT ![p] = s]
    /\ UNCHANGED <<status, peerNews, ourNews, owed, matched, settled, stalled, lru>>

\* Lease expiry or disposal (on_spdp_peer_removed()), in either tier.
Leave(p) ==
    /\ alive[p]
    /\ Spend
    /\ alive' = [alive EXCEPT ![p] = FALSE]
    /\ status' = [status EXCEPT ![p] = "unknown"]
    /\ peerNews' = [peerNews EXCEPT ![p] = FALSE]
    /\ ourNews' = [ourNews EXCEPT ![p] = FALSE]
    /\ owed' = [owed EXCEPT ![p] = FALSE]
    /\ shares' = [shares EXCEPT ![p] = FALSE]
    /\ matched' = [matched EXCEPT ![p] = FALSE]
    /\ settled' = [settled EXCEPT ![p] = FALSE]
    /\ stalled' = [stalled EXCEPT ![p] = FALSE]
    /\ lru' = Without(lru, p)

\* The peer creates or deletes an endpoint.
PeerAnnounces(p) ==
    /\ alive[p]
    /\ Spend
    /\ peerNews' = [peerNews EXCEPT ![p] = TRUE]
    /\ \E s \in BOOLEAN : shares' = [shares EXCEPT ![p] = s]
    /\ UNCHANGED <<alive, status, ourNews, owed, matched, settled, stalled, lru>>

\* A local endpoint is created or deleted: active peers receive it through
\* their proxies, parked peers are owed it, and any peer may start or stop
\* sharing a match.
LocalChange ==
    /\ Spend
    /\ ourNews' = [p \in Peers |-> ourNews[p] \/ status[p] = "active"]
    /\ owed' = [p \in Peers |-> owed[p] \/ status[p] = "parked"]
    /\ \E s \in [Peers -> BOOLEAN] : shares' = [p \in Peers |-> alive[p] /\ s[p]]
    /\ settled' = [p \in Peers |-> settled[p] /\ status[p] # "active"]
    /\ UNCHANGED <<alive, status, peerNews, matched, stalled, lru>>

(***************************************************************************)
(* This side.                                                               *)
(***************************************************************************)

\* An SPDP announcement: a new peer gets a record if one is free, and a
\* pending one asks for a slot (on_spdp_new_peer()). A parked peer's
\* announcement only renews its lease.
SpdpAnnouncement(p) ==
    /\ alive[p]
    /\ status[p] \in {"unknown", "pending"}
    /\ status[p] = "unknown" => Cardinality(Known) < Records
    /\ Admit(p, [status EXCEPT ![p] = "pending"])
    /\ UNCHANGED <<alive, shares, budget>>

\* A parked peer's SEDP HEARTBEAT or DATA above its parked number
\* (on_announcer_traffic()); repeated with every HEARTBEAT until admitted.
ParkedTraffic(p) ==
    /\ alive[p]
    /\ status[p] = "parked"
    /\ peerNews[p]
    /\ Admit(p, status)
    /\ UNCHANGED <<alive, shares, budget>>

\* serve_owed_peer(): one owed peer, chosen by ServeOrder.
ServedNext(p) ==
    /\ owed[p]
    /\ ServeOrder = "table" => \A r \in 1..(p - 1) : ~owed[r]

ServeOwed(p) ==
    /\ ServedNext(p)
    /\ Admit(p, status)
    /\ UNCHANGED <<alive, shares, budget>>

\* Something announced on either side is not yet processed, or the match
\* table is out of date.
ExchangePending(p) == peerNews[p] \/ ourNews[p] \/ matched[p] # shares[p]

\* The SEDP exchange with an active peer completes. DATA from the peer moves
\* it to the young end of the oldest-first order (lastSedpMs).
Exchange(p) ==
    /\ status[p] = "active"
    /\ p \notin Unreachable
    /\ ExchangePending(p)
    /\ peerNews' = [peerNews EXCEPT ![p] = FALSE]
    /\ ourNews' = [ourNews EXCEPT ![p] = FALSE]
    /\ matched' = [matched EXCEPT ![p] = shares[p]]
    /\ settled' = [settled EXCEPT ![p] = FALSE]
    /\ stalled' = [stalled EXCEPT ![p] = FALSE]
    /\ lru' = IF peerNews[p] THEN Append(Without(lru, p), p) ELSE lru
    /\ UNCHANGED <<alive, status, owed, shares, budget>>

\* peer_is_settled(): the settle timer runs out with nothing to acknowledge
\* and no local match.
Settle(p) ==
    /\ status[p] = "active"
    /\ ~settled[p]
    /\ ~ourNews[p]
    /\ ~matched[p]
    /\ settled' = [settled EXCEPT ![p] = TRUE]
    /\ UNCHANGED <<alive, status, peerNews, ourNews, owed, shares, matched, stalled, lru, budget>>

\* peer_is_stalled(): the exchange deadline runs out on an active peer with
\* no local match that cannot settle, because this side's announcements
\* are still unacknowledged. The deadline outlasts any exchange that can
\* complete, so a reachable peer stalls only once its exchange is done,
\* and then it can settle instead: only an unreachable peer stalls.
Stall(p) ==
    /\ StallRule = "deadline"
    /\ status[p] = "active"
    /\ ~settled[p] /\ ~stalled[p] /\ ~matched[p]
    /\ ourNews[p]
    /\ p \notin Unreachable => ~ExchangePending(p)
    /\ stalled' = [stalled EXCEPT ![p] = TRUE]
    /\ UNCHANGED <<alive, status, peerNews, ourNews, owed, shares, matched, settled, lru, budget>>

Environment ==
    \/ \E p \in Peers : Join(p) \/ Leave(p) \/ PeerAnnounces(p)
    \/ LocalChange

Next ==
    \/ Environment
    \/ \E p \in Peers :
        SpdpAnnouncement(p) \/ ParkedTraffic(p) \/ Exchange(p) \/ Settle(p) \/ Stall(p)
        \/ ServeOwed(p)

Spec == Init /\ [][Next]_vars

\* SPDP is periodic, HEARTBEATs repeat, the tick runs, and reachable peers'
\* exchanges complete.
FairSpec ==
    /\ Spec
    /\ \A p \in Peers :
        /\ SF_vars(SpdpAnnouncement(p))
        /\ SF_vars(ParkedTraffic(p))
        /\ WF_vars(Exchange(p))
        /\ WF_vars(Settle(p))
        /\ WF_vars(Stall(p))
        /\ SF_vars(ServeOwed(p))

(***************************************************************************)
(* Properties.                                                             *)
(***************************************************************************)
SlotsBounded == Cardinality(Active) <= Slots
LruIsActive == SeqSet(lru) = Active /\ Len(lru) = Cardinality(Active)
ParkedUnmatched == \A p \in Peers : status[p] = "parked" => ~matched[p]

\* A peer with a local match is never parked to make room.
NeverParkMatched ==
    [][\A p \in Peers : status[p] = "active" /\ status'[p] = "parked" => ~matched[p]]_vars

\* The board has room: fewer live peers share a match than there are slots,
\* so one slot is always left to take turns in. A board with more is over
\* capacity, which the design accepts (a matched peer is never parked).
Fits == Cardinality({p \in Peers : alive[p] /\ shares[p]}) < Slots

\* Once the network is quiet, and if the board has room, every live peer
\* that can be reached has been exchanged with: it is active, or parked
\* with nothing new on either side.
EveryPeerServed ==
    <>[]Fits =>
        <>[](\A p \in Peers \ Unreachable : alive[p] =>
                \/ status[p] = "active" /\ ~peerNews[p] /\ ~ourNews[p]
                \/ status[p] = "parked" /\ ~peerNews[p] /\ ~owed[p])

\* Once the network is quiet, and if the board has room, every live peer
\* that can be reached and shares a match with a local endpoint is matched.
EveryMatchMade ==
    <>[]Fits => <>[](\A p \in Peers \ Unreachable : alive[p] /\ shares[p] => matched[p])

=============================================================================
