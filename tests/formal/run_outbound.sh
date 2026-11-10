#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
#
# Checks one property of RosettePeerOutbound with TLC.
#
# Usage: tests/formal/run_outbound.sh <settle_rule> <acknack_retry> <property> [versions] [announcements]
#   settle_rule    code | heard_writer
#   acknack_retry  FALSE (the code) | TRUE
#   property       NoParkUninformed (action) | EventuallyInformed | EventuallyParkable (liveness)
set -euo pipefail

settle=$1 retry=$2 property=$3 versions=${4:-3} announce=${5:-2}
here=$(cd "$(dirname "$0")" && pwd)
jar_dir=${TLA_TOOLS_DIR:-$HOME/.local/share/tlaplus/v1.7.4}
jar_sha=936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88
echo "$jar_sha  $jar_dir/tla2tools.jar" | sha256sum --check --quiet

spec=FairSpec
[ "$property" = NoParkUninformed ] && spec=Spec

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cp "$here"/*.tla "$work"/
cat > "$work/Outbound.cfg" <<CFG
CONSTANTS
    MaxVersion = $versions
    MaxAnnounce = $announce
    SettleRule = "$settle"
    AcknackRetry = $retry
SPECIFICATION $spec
PROPERTY $property
CHECK_DEADLOCK FALSE
CFG

docker run --rm -v "$work":/model -v "$jar_dir":/tla -w /model eclipse-temurin:21-jre \
    java -XX:+UseParallelGC -Xmx6g -cp /tla/tla2tools.jar tlc2.TLC \
    -workers auto -config Outbound.cfg -cleanup RosettePeerOutbound.tla
