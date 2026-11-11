#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
#
# Checks properties of RosetteReaderDelivery with TLC.
#
# Usage: tests/formal/run_delivery.sh <property>[,<property>...] [key=value ...]
#   property  InOrder | NoAckPastUndelivered | WithinCap | HoldsInWindow | SkipOnlyKeepLast
#             | TypeOK (invariants) | EveryChangeDone (liveness)
#             | NeverHoldsCap | NeverPoolFull | NeverRefusedPastWindow | NeverRefusedForPool
#             | NeverEvictedAcrossReaders (witnesses: each must be reported violated)
#   keys (default):
#     readers (2)  changes (2)  cap (1)  slots (1)  blocks (1)  depth (0 = KEEP_ALL)
#     budget (2)   deliver (in_order | on_arrival)  admit (window | any)
#     block_rule (evict_any | evict_own | drop_new)  silent (FALSE | TRUE)
#     ack_reserve (TRUE | FALSE)
#   The defaults are the code (window admission, cross-reader block
#   eviction, a block kept for the ACKNACK) at the smallest size whose
#   witnesses are all reported violated, so every rule is exercised.
#
# Expected:
#   - Each Never* witness is reported violated for the defaults and for
#     changes=4 cap=2 slots=2 blocks=3; a witness that holds means that
#     configuration never reaches its rule.
#   - Every invariant holds for those two configurations; InOrder fails
#     with deliver=on_arrival.
#   - EveryChangeDone holds for the defaults, with admit=any, and with
#     silent=TRUE: the change that closes a gap needs no slot, takes a
#     block from whichever reader holds one, and can always be requested.
#   - With ack_reserve=FALSE it fails: held changes take every block, the
#     reader cannot answer a HEARTBEAT, and its gap is never requested.
#   - With ack_reserve=FALSE block_rule=drop_new it fails as well: the
#     change that closes a gap finds no block and every resend is dropped.
#   - With block_rule=evict_own and silent=TRUE it fails: writer 1 stops
#     with reader 1 holding blocks, and reader 2's gap never closes.
#     It holds with silent=FALSE.
#   - At budget=1, safety takes hours and liveness longer than a day on
#     four cores; changes=2 runs in minutes but never reaches the window
#     rule (its witnesses hold).
set -euo pipefail

property=$1
shift
readers=2 changes=3 cap=1 slots=1 blocks=2 depth=0 budget=1
deliver=in_order admit=window block_rule=evict_any silent=FALSE ack_reserve=TRUE
for arg in "$@"; do
    case $arg in
        readers=* | changes=* | cap=* | slots=* | blocks=* | depth=* | budget=* \
            | deliver=* | admit=* | block_rule=* | silent=* | ack_reserve=*)
            declare "$arg"
            ;;
        *)
            echo "unknown argument: $arg" >&2
            exit 2
            ;;
    esac
done

here=$(cd "$(dirname "$0")" && pwd)
jar_dir=${TLA_TOOLS_DIR:-$HOME/.local/share/tlaplus/v1.7.4}
jar_sha=936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88
echo "$jar_sha  $jar_dir/tla2tools.jar" | sha256sum --check --quiet

check="" spec=Spec
for name in ${property//,/ }; do
    case $name in
        Every*) check+="PROPERTY $name"$'\n' spec=FairSpec ;;
        NeverRefused* | NeverEvicted*) check+="PROPERTY $name"$'\n' ;;
        *) check+="INVARIANT $name"$'\n' ;;
    esac
done

# TLC_WORK keeps the model directory, and TLC's checkpoints in it, so a
# long run interrupted can resume with TLC_RECOVER=<states/dir name>.
if [[ -n ${TLC_WORK:-} ]]; then
    work=$TLC_WORK
    mkdir -p "$work"
else
    work=$(mktemp -d)
    trap 'rm -rf "$work"' EXIT
fi
cp "$here"/RosetteReaderDelivery.tla "$work"/
cat > "$work/Delivery.cfg" <<CFG
CONSTANTS
    Readers = $readers
    N = $changes
    Cap = $cap
    Slots = $slots
    Blocks = $blocks
    Depth = $depth
    Budget = $budget
    DeliverRule = "$deliver"
    AdmitRule = "$admit"
    BlockRule = "$block_rule"
    SilentWriter = $silent
    AckReserve = $ack_reserve
SPECIFICATION $spec
$check
CHECK_DEADLOCK FALSE
CFG

docker run --rm -v "$work":/model -v "$jar_dir":/tla -w /model eclipse-temurin:21-jre \
    java -XX:+UseParallelGC -Xmx"${TLC_HEAP:-6g}" -cp /tla/tla2tools.jar tlc2.TLC \
    -workers auto -config Delivery.cfg -checkpoint 30 \
    ${TLC_RECOVER:+-recover states/$TLC_RECOVER} ${TLC_RECOVER:--cleanup} RosetteReaderDelivery.tla
