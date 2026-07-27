"""
insert_sites.py

Reads the generated force-site lines from models/lizard_sites.xml and inserts
each body's sites into the matching <!-- SITES_<Body> --> placeholder inside
Lizard_Sand.xml.

Run from any working directory:
    python scripts/insert_sites.py
"""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

sites_file = os.path.join(ROOT, 'models', 'lizard_sites.xml')
base_file  = os.path.join(ROOT, 'Lizard_Sand.xml')
output_file = os.path.join(ROOT, 'Lizard_Sand.xml')   # overwrite in place

bodies = ['Mid', 'Front', 'FR', 'FL', 'Back', 'HR', 'HL', 'Tail']

# 1. Read all generated site lines
with open(sites_file, 'r') as f:
    all_sites_text = f.read()

# 2. Split the site lines by body using the comment headers
#    Each block in lizard_sites.xml looks like:
#       <!-- ===== Sites for FR (2000 triangles) ===== -->
#       <site name="force_FR_site_0" .../>
#       ...
sites_by_body = {}
for body in bodies:
    # grab every line that contains force_<body>_site_
    pattern = re.compile(rf'<site name="force_{body}_site_\d+".*?/>')
    matches = pattern.findall(all_sites_text)
    sites_by_body[body] = matches
    print(f"  {body}: {len(matches)} sites found")

# 3. Read the base Lizard_Sand.xml
with open(base_file, 'r') as f:
    xml = f.read()

# 4. Replace each placeholder with that body's sites (indented for readability)
for body in bodies:
    placeholder = f'<!-- SITES_{body} -->'
    if placeholder not in xml:
        print(f"  WARNING: placeholder {placeholder} not found in Lizard_Sand.xml!")
        continue
    indented = '\n'.join('      ' + line for line in sites_by_body[body])
    xml = xml.replace(placeholder, indented)

# 5. Write the result
with open(output_file, 'w') as f:
    f.write(xml)

total = sum(len(v) for v in sites_by_body.values())
print(f"\nDone! Inserted {total} sites total into {output_file}")
