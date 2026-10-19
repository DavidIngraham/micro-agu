import json
from run import P,run
spec=dict(id='candidate-switch',variant='candidate',weather='default',seed=73,switchTest=True)
(P/'switch-protocol.json').write_text(json.dumps([spec],indent=2))
(P/'switch-execution.json').write_text(json.dumps([run(spec)],indent=2))
