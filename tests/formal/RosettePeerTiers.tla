-------------------------- MODULE RosettePeerTiers --------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
(***************************************************************************)
(* Rosette's choices on top of SedpReliable: the two-tier peer table and  *)
(* the hosted graph cache's refresh of a reactivated peer.                *)
(*                                                                         *)
(* One remote peer is either active (its SEDP announcer is matched to a   *)
(* WriterProxy) or parked (no proxy; the highest number resolved at       *)
(* parking is kept). The model follows the code:                          *)
(*                                                                         *)
(*  - Parking: RTPSParticipant::park_peer(), allowed once the peer has    *)
(*    settled (RTPS_PEER_SETTLE_MS without SEDP DATA, an abstract timer   *)
(*    here) and its reader is caught up (SedpReader::caught_up_with()).   *)
(*    SettleRule = "heard_writer", the code, also requires that the proxy *)
(*    has heard a HEARTBEAT, DATA or GAP since activation               *)
(*    (SedpReader::heard_from()); "caught_up" does not, so a fresh proxy  *)
(*    that has heard nothing counts as caught up. The announcer's         *)
(*    periodic HEARTBEAT stands in for the preemptive ACKNACK retry that *)
(*    makes the peer answer, which RosettePeerOutbound checks.            *)
(*  - Reactivation: by a DATA or HEARTBEAT above the kept number          *)
(*    (on_announcer_traffic(), which then routes the message to the new   *)
(*    proxy), or at any time because this side owes the peer something   *)
(*    (serve_owed_peer()). The new WriterProxy starts empty.              *)
(*  - Delivery: Delivery = "on_arrival" is the code (StatefulReader       *)
(*    delivers any unresolved DATA at once, WriterProxy::is_new_change()).*)
(*    "in_order" is the specification's reader, which exposes changes    *)
(*    only up to available_changes_max (§8.4.10.4); comparing the two     *)
(*    shows what delivering on arrival costs.                             *)
(*  - Refresh: activate_peer() marks the cache's entries unconfirmed when *)
(*    the kept number is not 0; finish_peer_refreshes() sweeps those      *)
(*    still unconfirmed once SweepRule holds. SweepRule = "caught_up" is  *)
(*    the code; "post_activation_hb" adds the condition proposed in      *)
(*    review on PR #8: a HEARTBEAT received since reactivation.           *)
(*  - Parking during a refresh: ParkRule = "keep_refresh_target" is the   *)
(*    code, which keeps the larger of the refresh's target and the        *)
(*    reader's available_changes_max, so the next activation resumes the  *)
(*    refresh; "resolved_only" keeps available_changes_max alone, which    *)
(*    abandons the refresh when the new proxy has resolved less.          *)
(*                                                                         *)
(* Time enters in one place: a datagram's delay is shorter than            *)
(* RTPS_PEER_SETTLE_MS, so nothing sent before the peer settled is still   *)
(* in flight when it is parked. Every message the model reorders was sent  *)
(* within one settle period of the one it overtakes.                       *)
(*                                                                         *)
(* The graph cache (rmw_rosette::GraphCache) maps each endpoint to absent, *)
(* confirmed or unconfirmed; a listed endpoint is one that is not absent. *)
(***************************************************************************)
EXTENDS SedpReliable

CONSTANTS
    Delivery,    \* "on_arrival" (the code) or "in_order" (§8.4.10.4).
    SweepRule,   \* "caught_up" (the code) or "post_activation_hb".
    ParkRule,    \* "keep_refresh_target" (the code) or "resolved_only".
    SettleRule   \* "heard_writer" (the code) or "caught_up".

ASSUME Delivery \in {"on_arrival", "in_order"}
ASSUME SweepRule \in {"caught_up", "post_activation_hb"}
ASSUME ParkRule \in {"keep_refresh_target", "resolved_only"}
ASSUME SettleRule \in {"heard_writer", "caught_up"}

VARIABLES
    tier,        \* "active" or "parked".
    proxy,       \* The WriterProxy while active; FreshProxy while parked.
    kept,        \* PeerEntry::parkedPublicationsSn: kept number, or refresh target.
    settled,     \* The settle timer has run out since the last DATA or activation.
    heard,       \* WriterProxy::heard: a HEARTBEAT, DATA or GAP since activation.
    delivered,   \* in_order only: changes delivered up to this number.
    cache,       \* [Endpoints -> {"absent", "confirmed", "unconfirmed"}]
    disposedSeen \* Ghost: endpoints whose disposal this side has delivered.

localVars == <<tier, proxy, kept, settled, heard, delivered, cache, disposedSeen>>
vars == <<written, wsn, irrelevant, withdrawn, net, tier, proxy, kept, settled, heard, delivered,
          cache, disposedSeen>>

Listed(e) == cache[e] # "absent"
ListedSet == {e \in Endpoints : Listed(e)}

TypeOK ==
    /\ PeerTypeOK
    /\ tier \in {"active", "parked"}
    /\ proxy \in Proxies
    /\ kept \in 0..MaxSN
    /\ settled \in BOOLEAN
    /\ heard \in BOOLEAN
    /\ delivered \in 0..MaxSN
    /\ cache \in [Endpoints -> {"absent", "confirmed", "unconfirmed"}]
    /\ disposedSeen \subseteq Endpoints

Init ==
    /\ PeerInit
    /\ tier = "active"
    /\ proxy = FreshProxy
    /\ kept = 0
    /\ settled = FALSE
    /\ heard = FALSE
    /\ delivered = 0
    /\ cache = [e \in Endpoints |-> "absent"]
    /\ disposedSeen = {}

(***************************************************************************)
(* Delivering one change to SEDP and the graph cache: an announcement     *)
(* records or confirms the endpoint (GraphCache::add()), a disposal       *)
(* forgets it (GraphCache::remove()).                                      *)
(***************************************************************************)
Apply(c, ch) == [c EXCEPT ![ch.ep] = IF ch.kind = "alive" THEN "confirmed" ELSE "absent"]
DisposalsOf(sns) == {written[s].ep : s \in {x \in sns : written[x].kind = "dispose"}}

RECURSIVE ApplyInOrder(_, _, _, _)
ApplyInOrder(c, p, s, hi) ==
    IF s > hi THEN c
    ELSE ApplyInOrder(IF p.st[s] = "received" THEN Apply(c, written[s]) ELSE c, p, s + 1, hi)

\* The cache and ghost after proxy `p2` replaced `p1`, which received `newSN`
\* (0 for a GAP or HEARTBEAT) on top of `from`, delivered up to `d`.
DeliverAfter(p2, newSN, from, d) ==
    IF Delivery = "on_arrival"
    THEN [c |-> IF newSN = 0 THEN from ELSE Apply(from, written[newSN]),
          d |-> d,
          seen |-> IF newSN = 0 THEN {} ELSE DisposalsOf({newSN})]
    ELSE LET hi == AvailMax(p2)
         IN [c |-> ApplyInOrder(from, p2, d + 1, hi),
             d |-> Max(d, hi),
             seen |-> DisposalsOf({s \in (d + 1)..hi : p2.st[s] = "received"})]

\* How the proxy takes one message.
Absorb(p, m) ==
    CASE m.type = "DATA" -> IF Resolved(p, m.sn) THEN p ELSE ReceivedChangeSet(p, m.sn)
      [] m.type = "GAP"  -> IrrelevantChangeSet(p, m.lo, m.hi)
      [] m.type = "HB"   -> HeartbeatUpdate(p, m.first, m.last)

\* The SN a message delivers on arrival, or 0.
NewData(p, m) == IF m.type = "DATA" /\ ~Resolved(p, m.sn) THEN m.sn ELSE 0

(***************************************************************************)
(* Reactivation state, shared by both triggers: activate_peer() starts a  *)
(* refresh when the kept number is not 0 (GraphCache::begin_refresh()).  *)
(***************************************************************************)
Unconfirm(c) == [e \in Endpoints |-> IF c[e] = "absent" THEN "absent" ELSE "unconfirmed"]
CacheOnActivation == IF kept > 0 THEN Unconfirm(cache) ELSE cache

\* Take a message from the network, possibly keeping a duplicate in flight.
Consume(m) == \E dup \in BOOLEAN : net' = IF dup THEN net ELSE net \ {m}

Triggers(m) ==
    \/ m.type = "DATA" /\ m.sn > kept
    \/ m.type = "HB" /\ m.last > kept

\* An active peer's message reaches its WriterProxy and SEDP.
TakeActive(m) ==
    /\ tier = "active"
    /\ LET p2 == Absorb(proxy, m)
           out == DeliverAfter(p2, NewData(proxy, m), cache, delivered)
       IN /\ proxy' = p2
          /\ cache' = out.c
          /\ delivered' = out.d
          /\ disposedSeen' = disposedSeen \cup out.seen
          /\ settled' = IF m.type = "DATA" THEN FALSE ELSE settled
          /\ heard' = TRUE
    /\ UNCHANGED <<peerVars, tier, kept>>

\* A parked peer's message: above the kept number it reactivates the peer and
\* is routed to the new proxy; otherwise nothing takes it.
TakeParked(m) ==
    /\ tier = "parked"
    /\ IF Triggers(m)
       THEN LET p2 == Absorb(FreshProxy, m)
                out == DeliverAfter(p2, NewData(FreshProxy, m), CacheOnActivation, 0)
            IN /\ tier' = "active"
               /\ proxy' = p2
               /\ cache' = out.c
               /\ delivered' = out.d
               /\ disposedSeen' = disposedSeen \cup out.seen
               /\ settled' = FALSE
               /\ heard' = TRUE
               /\ UNCHANGED kept
       ELSE UNCHANGED localVars
    /\ UNCHANGED peerVars

Receive(m) == Consume(m) /\ (TakeActive(m) \/ TakeParked(m))

\* A datagram the announcer sends now and the network does not lose: the
\* step fairness rests on, since the network may lose any message in flight.
Transmit(m) == Sendable(m) /\ UNCHANGED net /\ (TakeActive(m) \/ TakeParked(m))

\* serve_owed_peer(): this side's announcements changed, so it reactivates
\* the peer without any message from it.
OwedReactivate ==
    /\ tier = "parked"
    /\ tier' = "active"
    /\ proxy' = FreshProxy
    /\ cache' = CacheOnActivation
    /\ delivered' = 0
    /\ settled' = FALSE
    /\ heard' = FALSE
    /\ UNCHANGED <<peerVars, net, kept, disposedSeen>>

SettleTimerExpires ==
    /\ tier = "active"
    /\ ~settled
    /\ settled' = TRUE
    /\ UNCHANGED <<peerVars, net, tier, proxy, kept, heard, delivered, cache, disposedSeen>>

\* park_peer(): the kept number is the reader's available_changes_max, and
\* under "keep_refresh_target" no less than an unfinished refresh's target.
Park ==
    /\ tier = "active"
    /\ settled
    /\ ~HasMissing(proxy)
    /\ SettleRule = "heard_writer" => heard
    /\ net = {}
    /\ tier' = "parked"
    /\ kept' = IF ParkRule = "resolved_only" THEN AvailMax(proxy) ELSE Max(kept, AvailMax(proxy))
    /\ proxy' = FreshProxy
    /\ delivered' = 0
    /\ settled' = FALSE
    /\ heard' = FALSE
    /\ UNCHANGED <<peerVars, net, cache, disposedSeen>>

SweepReady ==
    /\ ~HasMissing(proxy)
    /\ AvailMax(proxy) >= kept
    /\ SweepRule = "post_activation_hb" => proxy.hbSeen

\* finish_peer_refreshes() in tick(): GraphCache::end_refresh().
FinishRefresh ==
    /\ tier = "active"
    /\ kept > 0
    /\ SweepReady
    /\ cache' = [e \in Endpoints |-> IF cache[e] = "unconfirmed" THEN "absent" ELSE cache[e]]
    /\ kept' = 0
    /\ UNCHANGED <<peerVars, net, tier, proxy, settled, heard, delivered, disposedSeen>>

\* The peer drops a disposal once this side has acknowledged it. Parking is
\* local: the peer keeps this side's SEDP reader matched, so what counts is
\* the last ACKNACK base, the kept number while parked and the reader's
\* available_changes_max while active.
HeldByReader(s) == IF tier = "parked" THEN s <= kept ELSE s <= AvailMax(proxy)

Next ==
    \/ PeerNext /\ UNCHANGED localVars
    \/ \E s \in SNs : PurgeDisposal(s, HeldByReader(s)) /\ UNCHANGED localVars
    \/ \E m \in net : Receive(m)
    \/ \E m \in Messages : Transmit(m)
    \/ OwedReactivate
    \/ SettleTimerExpires
    \/ Park
    \/ FinishRefresh

Spec == Init /\ [][Next]_vars

\* The reader's tick runs, and every message the announcer keeps offering
\* (a periodic HEARTBEAT, a repair DATA or GAP) eventually gets through.
FairSpec == Spec /\ WF_vars(FinishRefresh) /\ \A m \in Messages : WF_vars(Transmit(m))

(***************************************************************************)
(* Properties.                                                             *)
(***************************************************************************)

\* A listed endpoint leaves the listing only when the peer no longer has it.
NoLiveDrop ==
    [][\A e \in Endpoints : (Listed(e) /\ cache'[e] = "absent") => e \notin PeerLive']_vars

\* Once this side has delivered an endpoint's disposal, it never lists it
\* again (GUIDs are not reused, so it can only be stale).
NoListAfterDisposal == \A e \in disposedSeen : ~Listed(e)

\* The peer's writes are bounded, so every behavior goes quiet; the listing
\* then settles on exactly the endpoints the peer has.
EventuallyConsistent == <>[](ListedSet = PeerLive)

\* Unconfirmed entries exist only while a refresh is under way.
UnconfirmedOnlyInRefresh == (\E e \in Endpoints : cache[e] = "unconfirmed") => kept > 0

=============================================================================
