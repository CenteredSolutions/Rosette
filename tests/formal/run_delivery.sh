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
#             | NeverEvictedAcrossReaders | NeverRefusedForAck (witnesses)
#   keys (default):
#     readers (2)  changes (3)  cap (1)  slots (1)  sample_blocks (1)  control_blocks (1)
#     depth (0 = KEEP_ALL)  budget (1)  deliver (in_order | on_arrival)  admit (window | any)
#     block_rule (evict_any | evict_own | drop_new)  silent (FALSE | TRUE)
#     ack_reserve (TRUE | FALSE)
#   The defaults are the code (window admission, cross-reader block
#   eviction, a block kept for the ACKNACK) at the smallest size where
#   held changes can pin every sample block while an ACKNACK still finds a
#   control block, so the change that closes a gap must evict.
#
# Configurations:
#   A  the defaults.
#   B  slots=2 sample_blocks=2 control_blocks=0: the small classes are
#      drained, so the ACKNACK reserve keeps a sample block free.
#   C  changes=4 cap=2 slots=2 sample_blocks=2 control_blocks=1.
#
# Expected (witnesses stop at their first violation, in seconds):
#   - Every Never* witness except NeverRefusedForAck is reported violated
#     for A and C: the pool fills and the closing change evicts another
#     reader's held change.
#   - For A and C, NeverRefusedForAck holds: a free control block always
#     satisfies the reserve. For B, it is reported violated, and
#     NeverPoolFull, NeverRefusedForPool and NeverEvictedAcrossReaders
#     hold: the reserve keeps held changes below sample_blocks.
#   - InOrder fails with deliver=on_arrival.
#   - EveryChangeDone fails for B with ack_reserve=FALSE: held changes take
#     every block, the reader cannot answer a HEARTBEAT, and its gap is
#     never requested.
#   - It fails for A with block_rule=drop_new: one reader's held change pins
#     the only sample block, and every change that would close a gap is
#     dropped.
#   - It fails for A with block_rule=evict_own silent=TRUE: writer 1 stops
#     with reader 1 holding the sample block, and reader 2's gap never
#     closes.
#   - Every invariant holds for A, B and C; EveryChangeDone holds for A,
#     with admit=any, with silent=TRUE, with block_rule=evict_own, and
#     for B. These checks explore every state: even at budget=0 they pass
#     30 million distinct states and 3 GB of TLC state files, so run them
#     on a host with tens of GB free, with TLC_WORK on that disk.
set -euo pipefail

property=$1
shift
readers=2 changes=3 cap=1 slots=1 sample_blocks=1 control_blocks=1 depth=0 budget=1
deliver=in_order admit=window block_rule=evict_any silent=FALSE ack_reserve=TRUE
for arg in "$@"; do
    case $arg in
        readers=* | changes=* | cap=* | slots=* | sample_blocks=* | control_blocks=* \
            | depth=* | budget=* \
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
    SampleBlocks = $sample_blocks
    ControlBlocks = $control_blocks
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

# A resumed run reads its checkpoint; a fresh one clears earlier state
# directories. Exactly one of the two is passed.
if [[ -n ${TLC_RECOVER:-} ]]; then
    start_args=(-recover "states/$TLC_RECOVER")
else
    start_args=(-cleanup)
fi

docker run --rm --user "$(id -u):$(id -g)" -v "$work":/model -v "$jar_dir":/tla -w /model eclipse-temurin:21-jre \
    java -XX:+UseParallelGC -Xmx"${TLC_HEAP:-6g}" -cp /tla/tla2tools.jar tlc2.TLC \
    -workers auto -config Delivery.cfg -checkpoint 30 \
    "${start_args[@]}" RosetteReaderDelivery.tla
