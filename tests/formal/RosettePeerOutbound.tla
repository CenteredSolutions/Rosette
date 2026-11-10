------------------------- MODULE RosettePeerOutbound -------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
(***************************************************************************)
(* The outbound half of the two-tier peer table: whether a remote peer    *)
(* receives this participant's `ros_discovery_info` sample when it is     *)
(* owed one, across parking and reactivation.                              *)
(*                                                                         *)
(* One remote peer reads this side's SEDP announcements and its           *)
(* `ros_discovery_info` writer (every stock ROS 2 participant does). The  *)
(* model follows the code:                                                 *)
(*                                                                         *)
(*  - Reactivation (activate_peer()) matches the peer's builtin SEDP       *)
(*    endpoints from its SPDP data, so this side's SEDP writers have a     *)
(*    ReaderProxy for it at once and repair until it acknowledges (fair).  *)
(*  - Inbound, a fresh WriterProxy sends one preemptive, non-final ACKNACK *)
(*    (StatefulReader::send_preemptive_acknack()). Delivered, the peer     *)
(*    must answer it (§8.3.7.1), and the HEARTBEAT, NACK and repair that   *)
(*    follow are retried by both sides until done: the replay eventually   *)
(*    arrives. Lost, nothing follows: the peer's ReaderProxy for this side *)
(*    still holds everything acknowledged from before parking, so it sends *)
(*    no HEARTBEAT, and the reader sends ACKNACKs only in answer to one.   *)
(*    A new announcement by the peer does send a HEARTBEAT, which starts   *)
(*    the replay.                                                           *)
(*  - The discovery writer has a ReaderProxy for the peer only once the    *)
(*    replay has announced the peer's reader of it and matched it.         *)
(*    StatefulWriter::acknowledged_by_participant() holds for a peer with  *)
(*    no proxy.                                                             *)
(*  - Settled (peer_is_settled()), abstracting phase 2's conditions: this  *)
(*    side's SEDP is acknowledged, the SEDP reader is caught up (a proxy   *)
(*    that has heard of nothing is), and every user writer is acknowledged *)
(*    by the peer. Parking happens whenever the peer is settled, since    *)
(*    phase 2 shows another peer may need the slot at any time.            *)
(*  - A new discovery sample (discovery_info_changed()) is owed to a      *)
(*    parked peer, which is reactivated in turn (serve_owed_peer()).       *)
(*                                                                         *)
(* Rosette's choices, as constants:                                         *)
(*  - SettleRule = "code" settles as above; "heard_writer" also requires   *)
(*    that each SEDP reader proxy has heard a HEARTBEAT or DATA since      *)
(*    activation, so the replay has started.                               *)
(*  - AcknackRetry = FALSE is the code; TRUE resends the preemptive        *)
(*    ACKNACK until the writer is heard.                                   *)
(***************************************************************************)
EXTENDS Naturals

CONSTANTS
    MaxVersion,    \* Discovery samples written in a behavior.
    MaxAnnounce,   \* SEDP announcements by the peer in a behavior.
    SettleRule,    \* "code" or "heard_writer".
    AcknackRetry   \* BOOLEAN.

ASSUME SettleRule \in {"code", "heard_writer"} /\ AcknackRetry \in BOOLEAN

VARIABLES
    tier,          \* "active" or "parked".
    inbound,       \* "silent": nothing heard since activation and no ACKNACK in
                   \* flight; "asking": the preemptive ACKNACK is in flight;
                   \* "syncing": the peer is replaying; "synced".
    sedpAcked,     \* The peer acknowledged this side's SEDP announcements.
    discProxy,     \* The discovery writer has a ReaderProxy for the peer.
    version,       \* The discovery writer's current sample.
    held,          \* The sample the peer holds.
    owed,
    announced      \* Announcements the peer has made.

vars == <<tier, inbound, sedpAcked, discProxy, version, held, owed, announced>>

\* A new peer: its writers have never had this side acknowledge anything, so
\* they send HEARTBEATs unprompted and the replay starts on its own.
Init ==
    /\ tier = "active"
    /\ inbound = "syncing"
    /\ sedpAcked = FALSE
    /\ discProxy = FALSE
    /\ version = 1
    /\ held = 0
    /\ owed = FALSE
    /\ announced = 0

\* activate_peer(): a fresh proxy in each direction.
Activate ==
    /\ tier' = "active"
    /\ inbound' = "asking"
    /\ sedpAcked' = FALSE
    /\ discProxy' = FALSE
    /\ owed' = FALSE

(***************************************************************************)
(* The network and the peer.                                               *)
(***************************************************************************)
AcknackDelivered ==
    /\ tier = "active" /\ inbound = "asking"
    /\ inbound' = "syncing"
    /\ UNCHANGED <<tier, sedpAcked, discProxy, version, held, owed, announced>>

AcknackLost ==
    /\ tier = "active" /\ inbound = "asking"
    /\ inbound' = "silent"
    /\ UNCHANGED <<tier, sedpAcked, discProxy, version, held, owed, announced>>

\* The preemptive ACKNACK resent while nothing has been heard.
AcknackResent ==
    /\ AcknackRetry
    /\ tier = "active" /\ inbound = "silent"
    /\ inbound' = "asking"
    /\ UNCHANGED <<tier, sedpAcked, discProxy, version, held, owed, announced>>

\* The replay completes; the peer's reader of the discovery writer is matched.
ReplayDone ==
    /\ tier = "active" /\ inbound = "syncing"
    /\ inbound' = "synced"
    /\ discProxy' = TRUE
    /\ UNCHANGED <<tier, sedpAcked, version, held, owed, announced>>

\* The peer acknowledges this side's SEDP announcements.
SedpAcked ==
    /\ tier = "active" /\ ~sedpAcked
    /\ sedpAcked' = TRUE
    /\ UNCHANGED <<tier, inbound, discProxy, version, held, owed, announced>>

\* The discovery writer delivers its sample to a matched reader.
SampleDelivered ==
    /\ tier = "active" /\ discProxy /\ held # version
    /\ held' = version
    /\ UNCHANGED <<tier, inbound, sedpAcked, discProxy, version, owed, announced>>

\* The peer creates or deletes an endpoint: DATA and a HEARTBEAT follow.
PeerAnnounces ==
    /\ announced < MaxAnnounce
    /\ announced' = announced + 1
    /\ IF tier = "parked"
       THEN Activate /\ inbound' = "syncing"   \* on_announcer_traffic(), then routed
       ELSE /\ inbound' = IF inbound = "synced" THEN "synced" ELSE "syncing"
            /\ UNCHANGED <<tier, sedpAcked, discProxy, owed>>
    /\ UNCHANGED <<version, held>>

(***************************************************************************)
(* This side.                                                               *)
(***************************************************************************)
DiscoveryChange ==
    /\ version < MaxVersion
    /\ version' = version + 1
    /\ owed' = (tier = "parked")
    /\ UNCHANGED <<tier, inbound, sedpAcked, discProxy, held, announced>>

CaughtUp == inbound \in {"silent", "asking", "synced"}
DiscAcked == ~discProxy \/ held = version
Heard == inbound \in {"syncing", "synced"}

Settled ==
    /\ tier = "active"
    /\ sedpAcked
    /\ CaughtUp
    /\ DiscAcked
    /\ SettleRule = "heard_writer" => Heard

Park ==
    /\ Settled
    /\ tier' = "parked"
    /\ discProxy' = FALSE
    /\ owed' = FALSE
    /\ UNCHANGED <<inbound, sedpAcked, version, held, announced>>

ServeOwed ==
    /\ tier = "parked" /\ owed
    /\ Activate
    /\ UNCHANGED <<version, held, announced>>

Next ==
    \/ AcknackDelivered \/ AcknackLost \/ AcknackResent
    \/ ReplayDone \/ SedpAcked \/ SampleDelivered
    \/ PeerAnnounces \/ DiscoveryChange
    \/ Park \/ ServeOwed

Spec == Init /\ [][Next]_vars

\* Repairs complete, a resent ACKNACK eventually gets through, and owed peers
\* are served (phase 2: when the board has room).
FairSpec ==
    /\ Spec
    /\ WF_vars(ReplayDone) /\ WF_vars(SedpAcked) /\ WF_vars(SampleDelivered)
    /\ WF_vars(ServeOwed) /\ WF_vars(AcknackResent)
    /\ SF_vars(AcknackDelivered)

(***************************************************************************)
(* Properties.                                                             *)
(***************************************************************************)

\* A peer is never parked owing nothing while it lacks the current sample.
NoParkUninformed == [][tier = "active" /\ tier' = "parked" => held' = version' \/ owed']_vars

\* Once this side stops writing, the peer holds its current sample.
EventuallyInformed == <>[](held = version)

\* A peer whose replay can complete does not hold its slot forever.
EventuallyParkable == []<>(tier = "parked" \/ Settled)

=============================================================================
