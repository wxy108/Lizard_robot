"""
rebuild_sites.py

Rebuilds the force sites in Lizard_Sand.xml to match the CURRENT meshes.

It:
  1. removes all existing <site name="force_..._site_N" .../> lines
  2. re-inserts a <!-- SITES_<Body> --> placeholder after each body's geom
  3. fills each placeholder with fresh sites from models/lizard_sites.xml

Run from any working directory:
    python scripts/rebuild_sites.py

Prereq: run generate_sites.py first so models/lizard_sites.xml matches the
current meshes.
"""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
xml_path = os.path.join(ROOT, 'Lizard_Sand.xml')
sites_path = os.path.join(ROOT, 'models', 'lizard_sites.xml')

bodies = ['Mid', 'Front', 'FR', 'FL', 'Back', 'HR', 'HL', 'Tail']

# ---------- 1. read current XML ----------
with open(xml_path, 'r') as f:
    xml = f.read()

# ---------- 2. strip out all existing force_* site lines ----------
# remove any line that is a force site
before = len(xml)
xml = re.sub(r'[ \t]*<site name="force_[^"]+_site_\d+"[^/]*/>\n?', '', xml)
after = len(xml)
print(f"Removed old force-site lines ({before-after} chars).")

# also remove any leftover placeholders so we can re-add cleanly
for b in bodies:
    xml = xml.replace(f'<!-- SITES_{b} -->', '')

# ---------- 3. put a placeholder right after each body's <geom name="B" .../> ----------
for b in bodies:
    # match the geom line for this body (self-closing)
    pattern = re.compile(rf'(<geom name="{b}"[^>]*/>)')
    def repl(m):
        return m.group(1) + f'\n      <!-- SITES_{b} -->'
    xml, n = pattern.subn(repl, xml, count=1)
    if n == 0:
        print(f"  WARNING: couldn't find <geom name=\"{b}\" .../> to add placeholder")

# ---------- 4. read the freshly generated sites ----------
with open(sites_path, 'r') as f:
    all_sites = f.read()

sites_by_body = {}
for b in bodies:
    matches = re.findall(rf'<site name="force_{b}_site_\d+".*?/>', all_sites)
    sites_by_body[b] = matches
    print(f"  {b}: {len(matches)} new sites")

# ---------- 5. fill placeholders ----------
for b in bodies:
    placeholder = f'<!-- SITES_{b} -->'
    indented = '\n'.join('      ' + line for line in sites_by_body[b])
    xml = xml.replace(placeholder, indented)

# ---------- 6. write back ----------
with open(xml_path, 'w') as f:
    f.write(xml)

total = sum(len(v) for v in sites_by_body.values())
print(f"\nDone! Rebuilt Lizard_Sand.xml with {total} fresh sites.")
