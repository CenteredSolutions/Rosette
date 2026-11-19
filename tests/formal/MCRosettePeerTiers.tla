------------------------- MODULE MCRosettePeerTiers -------------------------
\* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
\* SPDX-License-Identifier: Apache-2.0
\* Model-checking harness for RosettePeerTiers: endpoints are interchangeable,
\* so TLC explores one ordering of them (safety properties only).
EXTENDS RosettePeerTiers, TLC

EndpointSymmetry == Permutations(Endpoints)
=============================================================================
