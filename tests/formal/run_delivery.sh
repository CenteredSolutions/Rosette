#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
#
# Checks one property of RosetteReaderDelivery with TLC.
#
# Usage: tests/formal/run_delivery.sh <property> [changes] [cap] [depth] [budget] [deliver_rule] [full_rule]
#   property      InOrder | NoAckPastUndelivered | WithinCap | SkipOnlyKeepLast | TypeOK (invariants)
#                 | EveryChangeDone (liveness)
#   depth         0 = KEEP_ALL, k = KEEP_LAST k
#   deliver_rule  in_order (the code) | on_arrival
#   full_rule     evict_highest (the code) | drop_new
#
# Expected (3 changes, budget 2 lost messages):
#   InOrder, NoAckPastUndelivered, WithinCap, SkipOnlyKeepLast hold for
#   cap 1 KEEP_ALL and cap 2 KEEP_LAST 2; InOrder fails with on_arrival.
#   EveryChangeDone holds with evict_highest for cap 1 and 2, KEEP_ALL and
#   KEEP_LAST 2. With drop_new it fails whenever the cap fills before
#   KEEP_LAST gives the gap up (cap 1 or 2 KEEP_ALL, cap 1 KEEP_LAST 2):
#   the change that closes the gap finds no block, is dropped, and every
#   resend of it is dropped too.
set -euo pipefail

property=$1 changes=${2:-3} cap=${3:-1} depth=${4:-0} budget=${5:-2}
deliver_rule=${6:-in_order} full_rule=${7:-evict_highest}
here=$(cd "$(dirname "$0")" && pwd)
jar_dir=${TLA_TOOLS_DIR:-$HOME/.local/share/tlaplus/v1.7.4}
jar_sha=936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88
echo "$jar_sha  $jar_dir/tla2tools.jar" | sha256sum --check --quiet

case $property in
    Every*) check="PROPERTY $property" spec=FairSpec ;;
    *) check="INVARIANT $property" spec=Spec ;;
esac

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cp "$here"/RosetteReaderDelivery.tla "$work"/
cat > "$work/Delivery.cfg" <<CFG
CONSTANTS
    N = $changes
    Cap = $cap
    Depth = $depth
    Budget = $budget
    DeliverRule = "$deliver_rule"
    FullRule = "$full_rule"
SPECIFICATION $spec
$check
CHECK_DEADLOCK FALSE
CFG

docker run --rm -v "$work":/model -v "$jar_dir":/tla -w /model eclipse-temurin:21-jre \
    java -XX:+UseParallelGC -Xmx6g -cp /tla/tla2tools.jar tlc2.TLC \
    -workers auto -config Delivery.cfg -cleanup RosetteReaderDelivery.tla
