"""Atomic episode directories. A manifest never advertises a partial episode."""
from pathlib import Path
import json
import os
import shutil
import tempfile
import numpy as np
from .dataset_reader import validate, export_video


def jsonable(v):
    if isinstance(v,np.ndarray): return v.tolist()
    if isinstance(v,np.generic): return v.item()
    if isinstance(v,dict): return {k:jsonable(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)): return [jsonable(x) for x in v]
    return v


class EpisodeRecorder:
    def __init__(self):
        self.observations=[]
        self.actions=[]
        self.joint_targets=[]
        self.trace=[]

    def append(self,result,trace):
        before,after=result['observation'],result['next_observation']
        if not self.observations:
            self.observations.append(before)
        elif before['physics_step']!=self.observations[-1]['physics_step']:
            raise ValueError('recorded transition discontinuity')
        self.actions.append(result['action'].copy())
        self.joint_targets.append(result['joint_target'].copy())
        self.observations.append(after)
        self.trace.append(jsonable(trace))

    def arrays(self):
        obs=self.observations
        out={f'observation.state.{k}':np.asarray([o[k] for o in obs]) for k in ('eef10','qpos','qvel')}
        out.update({'observation.images.head':np.asarray([o['head'] for o in obs]),
                    'action.eef10':np.asarray(self.actions),'action.joint_target':np.asarray(self.joint_targets),
                    'timestamp.sim':np.asarray([o['sim_time'] for o in obs]),
                    'control_tick':np.asarray([o['control_tick'] for o in obs]),
                    'physics_step':np.asarray([o['physics_step'] for o in obs]),
                    'done':np.r_[np.zeros(len(self.actions)-1,dtype=bool),True]})
        if 'wrist' in obs[0]:
            out['observation.images.wrist']=np.asarray([o['wrist'] for o in obs])
        for key in ('cube_pose','cube_velocity','finger_contact_forces'):
            out['oracle_debug.'+key]=np.asarray([o['oracle_debug'][key] for o in obs])
        return out

    def save(self,root,name,metadata,video=True):
        root=Path(root); root.mkdir(parents=True,exist_ok=True)
        destination=root/name
        if destination.exists(): raise FileExistsError(destination)
        arrays=self.arrays()
        metadata=jsonable({**metadata,'schema_version':'dynamicbench.eef10.columns.v1',
                  'wrist_status':'available' if 'observation.images.wrist' in arrays else 'not_available',
                  'control_hz':30,'physics_hz':300})
        summary=validate(arrays,metadata)
        temp=Path(tempfile.mkdtemp(prefix='.episode-',dir=root))
        try:
            np.savez_compressed(temp/'trajectory.npz',**arrays)
            (temp/'metadata.json').write_text(json.dumps(metadata,indent=2))
            (temp/'trace.jsonl').write_text(''.join(json.dumps(t)+'\n' for t in self.trace))
            if video: export_video(arrays,temp/'head.mp4')
            for p in temp.iterdir():
                with p.open('rb') as f: os.fsync(f.fileno())
            os.rename(temp,destination)
            manifest=root/'manifest.jsonl'
            previous=manifest.read_text() if manifest.exists() else ''
            entry=dict(episode=name,**summary,seed=metadata['seed'],belt_speed=metadata['belt_speed'],
                       source_commit=metadata['source_commit'],failure_reason=metadata['failure_reason'])
            fd,tmp=tempfile.mkstemp(prefix='.manifest-',dir=root)
            with os.fdopen(fd,'w') as f:
                f.write(previous+json.dumps(entry)+'\n'); f.flush(); os.fsync(f.fileno())
            os.replace(tmp,manifest)
            return entry
        except Exception:
            if temp.exists(): shutil.rmtree(temp)
            raise
