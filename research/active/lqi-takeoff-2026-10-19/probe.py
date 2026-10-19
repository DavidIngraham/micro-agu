import json
from run import P,run
spec=dict(id='candidate-headwind-probe',variant='candidate',weather='default',seed=73,north=-1.5,east=0)
(P/'probe-protocol.json').write_text(json.dumps([spec],indent=2))
(P/'probe-execution.json').write_text(json.dumps([run(spec)],indent=2))
