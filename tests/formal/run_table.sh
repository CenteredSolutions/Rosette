#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
#
# Checks one property of RosettePeerTable with TLC.
#
# Usage: tests/formal/run_table.sh <property> [peers] [slots] [records] [unreachable] [budget] [stall_rule] [serve_order]
#   property     SlotsBounded | LruIsActive | ParkedUnmatched | TypeOK (invariants)
#                | NeverParkMatched (action) | EveryPeerServed | EveryMatchMade (liveness)
#   unreachable  a TLA+ set of peer numbers, e.g. "{}" or "{1}"
#   stall_rule   deadline (the code) | none
#   serve_order  oldest (the code) | table
set -euo pipefail

liveness="" property=$1 peers=${2:-3} slots=${3:-2} records=${4:-$peers} unreachable=${5:-"{}"} budget=${6:-4} stall_rule=${7:-deadline} serve_order=${8:-oldest}
here=$(cd "$(dirname "$0")" && pwd)
jar_dir=${TLA_TOOLS_DIR:-$HOME/.local/share/tlaplus/v1.7.4}
jar_sha=936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88
echo "$jar_sha  $jar_dir/tla2tools.jar" | sha256sum --check --quiet

case $property in
    NeverParkMatched) check="PROPERTY $property" spec=Spec ;;
    Every*) check="PROPERTY $property" spec=FairSpec liveness="-lncheck final" ;;
    *) check="INVARIANT $property" spec=Spec ;;
esac

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cp "$here"/*.tla "$work"/
cat > "$work/Table.cfg" <<CFG
CONSTANTS
    N = $peers
    Slots = $slots
    Records = $records
    Unreachable = $unreachable
    Budget = $budget
    StallRule = "$stall_rule"
    ServeOrder = "$serve_order"
SPECIFICATION $spec
$check
CHECK_DEADLOCK FALSE
CFG

docker run --rm -v "$work":/model -v "$jar_dir":/tla -w /model eclipse-temurin:21-jre \
    java -XX:+UseParallelGC -Xmx6g -cp /tla/tla2tools.jar tlc2.TLC \
    -workers auto ${liveness} -config Table.cfg -cleanup RosettePeerTable.tla
