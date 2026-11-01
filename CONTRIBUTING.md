# Contributing to Rosette

Thank you for helping. This page covers the legal terms of a contribution and
the checks a pull request must pass.

## License of contributions

Rosette is licensed under the Apache License, Version 2.0. Any contribution
you submit is licensed under the same terms, as section 5 of the license
states:

> Unless You explicitly state otherwise, any Contribution intentionally
> submitted for inclusion in the Work by You to the Licensor shall be under the
> terms and conditions of this License, without any additional terms or
> conditions.

## Developer Certificate of Origin

Every commit must carry a `Signed-off-by:` line certifying the
[Developer Certificate of Origin 1.1](https://developercertificate.org/): that
you wrote the change or otherwise have the right to submit it under the
project's license. `git commit -s` adds the line. The same convention is used
across the ROS 2 core repositories.

## AI-assisted changes

Credit an assistant that drafted part of a commit with a `Co-Authored-By:`
trailer. You remain the author: sign off only on changes you have reviewed and
understand. See [AI_ASSISTANCE.md](AI_ASSISTANCE.md).

## Style and checks

[AGENTS.md](AGENTS.md) holds the coding style and the documentation rule
(`Verification:` and `References:` on every non-trivial function). Before
opening a pull request, run:

```bash
python3 -m pip install -r tools/style/requirements.txt
tools/style/run_style_checks.sh
```

Every source file starts with an SPDX header naming its copyright holder and
license. `tools/license/apply_license_headers.py` adds it to new files; formats
that cannot carry a comment are covered by [REUSE.toml](REUSE.toml).
