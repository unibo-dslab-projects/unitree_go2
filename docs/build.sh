#!/usr/bin/env bash
# builds the site into docs/_site; `./build.sh preview` serves it with live reload
set -euo pipefail

# ci reads this line too: a new quarkdown can rename the DOM the sketch script hooks into
QUARKDOWN_VERSION="2.4.1"

cd "$(dirname "$0")"

have=$(quarkdown --version 2>/dev/null | awk '/quarkdown version/ {print $NF}')
if [ "$have" != "$QUARKDOWN_VERSION" ]; then
    echo "quarkdown $have installed, but build.sh pins $QUARKDOWN_VERSION." >&2
    exit 1
fi

if [ "${1:-}" = "preview" ]; then
    exec quarkdown c main.qd --out _site/.stage --allow native-content --preview --watch
fi

rm -rf _site
quarkdown c main.qd --out _site/.stage --allow native-content
# quarkdown writes into <out>/<docname>/, but the site must sit at the pages root
cp -r _site/.stage/*/. _site/
rm -rf _site/.stage
