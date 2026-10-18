#!/usr/bin/env python3
"""Archive report-linked figures and their plotter source/dependencies, not logs/data."""
import argparse,ast,hashlib,json,re,shutil,html
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote,urlsplit
class Links(HTMLParser):
 def __init__(self):super().__init__();self.paths=[]
 def handle_starttag(self,tag,attrs):
  for key,value in attrs:
   if key in ('src','href') and value:self.paths.append(value)
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--native-source',type=Path);a=p.parse_args()
repo=Path(__file__).resolve().parents[1];out=repo/'research/archive/flight-log-analysis';source=a.source.resolve();figures={}
for report in source.rglob('*.html'):
 parser=Links();parser.feed(report.read_text(errors='replace'))
 for link in parser.paths:
  url=urlsplit(link)
  if url.scheme or url.netloc:continue
  path=(report.parent/unquote(url.path)).resolve()
  if path.suffix.lower() not in ('.png','.svg') or not path.is_file() or not path.is_relative_to(source):continue
  figures.setdefault(path,[]).append(str(report.relative_to(source)))
scripts=set()
for path in source.rglob('*.py'):
 text=path.read_text(errors='replace')
 if re.search(r'savefig|matplotlib|plotly|write_image',text):scripts.add(path)
# Include local Python imports so plotters retain their source dependencies.
pending=list(scripts)
while pending:
 path=pending.pop()
 try:tree=ast.parse(path.read_text(encoding='utf-8-sig'))
 except (SyntaxError,UnicodeError):continue
 names=[]
 for node in ast.walk(tree):
  if isinstance(node,ast.Import):names.extend(v.name.split('.')[0] for v in node.names)
  elif isinstance(node,ast.ImportFrom) and node.module:names.append(node.module.split('.')[0])
 for name in names:
  module=path.parent/(name+'.py')
  if module.is_file() and module not in scripts:scripts.add(module);pending.append(module)
entries=[]
def copy(path,relative,kind,**extra):
 data=path.read_bytes();dest=out/relative;dest.parent.mkdir(parents=True,exist_ok=True)
 if not dest.exists() or dest.read_bytes()!=data:dest.write_bytes(data)
 entries.append(dict(path=str(relative),source=str(path),bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),kind=kind,**extra))
for path,reports in sorted(figures.items()):copy(path,Path('figures')/path.relative_to(source),'figure',referenced_by=reports)
for path in sorted(scripts):copy(path,Path('scripts')/path.relative_to(source),'plotter_or_dependency')
if a.native_source:
 for name in ['plot_comparison.py','plot_damper.py','plot_longitudinal_isolation.py','longitudinal_frequency.py','analyze_longitudinal_study.py','compare_longitudinal_study.py']:
  path=a.native_source/name
  if path.is_file():copy(path,Path('scripts/native-sitl')/name,'native_plotter')
(out/'manifest.json').write_text(json.dumps(dict(source_root=str(source),policy='Report-linked PNG/SVG figures and plotting source only. Reports, logs, datasets, parameters and binary traces remain at their existing locations. Native plotters requiring model imports should run in the ArduPilot simulation package.',files=entries),indent=2))
print(json.dumps(dict(figures=len(figures),scripts=len(entries)-len(figures),bytes=sum(e['bytes'] for e in entries)),indent=2))

cards=[]
for entry in entries:
 if entry['kind']!='figure':continue
 url=entry['path'];label=html.escape(url.removeprefix('figures/'))
 cards.append(f'<figure><a href="{html.escape(url)}"><img loading="lazy" src="{html.escape(url)}" alt="{label}"></a><figcaption>{label}</figcaption></figure>')
(out/'index.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Flight research figures</title><style>body{font:16px/1.5 system-ui;margin:32px;color:#163042}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:20px}figure{margin:0;border:1px solid #ccd;padding:12px}img{width:100%}figcaption{overflow-wrap:anywhere}</style><h1>Flight research figures</h1><p>Final figures linked from the research reports, with plotting source in <code>scripts/</code>. Reports and datasets remain in Mission Planner; native SITL raw data remains in its original experiment directories. <a href="manifest.json">Source paths and SHA-256 manifest</a>.</p><main>'+''.join(cards)+'</main></html>')