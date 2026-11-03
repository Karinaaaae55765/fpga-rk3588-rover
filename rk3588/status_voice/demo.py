"""Independent voice test. Never imports a vehicle controller or opens a UART."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from status_voice.player import Player

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--play',action='store_true');p.add_argument('--device')
    p.add_argument('--prompt',default='test');p.add_argument('--all',action='store_true')
    a=p.parse_args();voice=Player('play' if a.play else 'dry',a.device)
    try:
        names=list(voice.prompts) if a.all else [a.prompt]
        for i,name in enumerate(names):
            if i:voice=Player('play' if a.play else 'dry',a.device)
            voice.submit(name);voice.close()
            state=voice.stats();print(json.dumps(dict(prompt=name,text=voice.prompts[name],**state),ensure_ascii=False))
            if state['failed'] or state['cancelled'] or state['dropped']:return 1
        return 0
    finally:voice.close()

if __name__=='__main__':sys.exit(main())
