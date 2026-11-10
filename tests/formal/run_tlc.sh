#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
#
# Checks one property of MCRosettePeerTiers under one variant with TLC.
#
# Usage: tests/formal/run_tlc.sh <delivery> <sweep_rule> <park_rule> <property> [max_sn] [net_cap]
#   delivery    on_arrival (the code) | in_order (DDSI-RTPS 2.5 §8.4.10.4)
#   sweep_rule  caught_up (the code)  | post_activation_hb
#   park_rule   code                  | keep_refresh_target
#   property    NoLiveDrop | NoListAfterDisposal | UnconfirmedOnlyInRefresh | TypeOK
#               | EventuallyConsistent (liveness: checked under FairSpec, without symmetry)
#
# TLC runs in a JRE container; tla2tools.jar is pinned by version and checksum.
set -euo pipefail

delivery=$1 sweep=$2 park=$3 property=$4 max_sn=${5:-4} net_cap=${6:-3}
here=$(cd "$(dirname "$0")" && pwd)
jar_dir=${TLA_TOOLS_DIR:-$HOME/.local/share/tlaplus/v1.7.4}
jar_sha=936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88
echo "$jar_sha  $jar_dir/tla2tools.jar" | sha256sum --check --quiet

spec=Spec symmetry="SYMMETRY EndpointSymmetry"
case $property in
    NoLiveDrop) check="PROPERTY $property" ;;
    EventuallyConsistent) check="PROPERTY $property" spec=FairSpec symmetry="" ;;
    *) check="INVARIANT $property" ;;
esac

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cp "$here"/*.tla "$work"/
cat > "$work/MC.cfg" <<CFG
CONSTANTS
    Endpoints = {e1, e2}
    MaxSN = $max_sn
    NetCap = $net_cap
    Delivery = "$delivery"
    SweepRule = "$sweep"
    ParkRule = "$park"
SPECIFICATION $spec
$symmetry
$check
CHECK_DEADLOCK FALSE
CFG

docker run --rm -v "$work":/model -v "$jar_dir":/tla -w /model eclipse-temurin:21-jre \
    java -XX:+UseParallelGC -Xmx6g -cp /tla/tla2tools.jar tlc2.TLC \
    -workers auto -config MC.cfg -cleanup MCRosettePeerTiers.tla
