"""Fit gyro scale from marked rotations, excluding idle thermal drift.

This never applies parameters. Reference angles are supplied by the operator;
wheel calibration is deliberately not computed from hand rotations.
"""
import argparse
import json
import math
from pathlib import Path
import statistics


def analyze(rows):
    if any(r['event'] in ('sample_gap','clock_went_backwards') for r in rows):
        raise ValueError('Incomplete sensor timeline; repeat reference recording')
    markers=[i for i,r in enumerate(rows) if r['event']=='mark']
    turns=[]
    excluded=[]
    for start,end in zip(markers,markers[1:]):
        first,last=rows[start],rows[end]
        reference=last['reference_deg']-first['reference_deg']
        if abs(reference)<30:
            continue
        samples=[r for r in rows[start+1:end] if r['event']=='sample']
        active=[r for r in samples if abs(r['raw_gyro_radps'])>.1]
        if not active:
            excluded.append({'start_s':first['elapsed_s'],'reason':'no rotation'})
            continue
        t0,t1=active[0]['stamp'],active[-1]['stamp']
        before=[r['raw_gyro_radps'] for r in samples if t0-5<=r['stamp']<t0-1]
        after=[r['raw_gyro_radps'] for r in samples if t1+1<r['stamp']<=t1+5]
        # Hand contact can produce isolated vibration spikes. Require at
        # least95% quiet samples and use the median, not a biased mean.
        if (len(before)<200 or len(after)<200 or
                any(sum(abs(v)<.02 for v in values)/len(values)<.95 for values in (before,after))):
            excluded.append({'start_s':first['elapsed_s'],'reason':'missing stationary brackets'})
            continue
        bias0,bias1=statistics.median(before),statistics.median(after)
        window=[r for r in samples if t0-1<=r['stamp']<=t1+1]
        angle=0.
        for a,b in zip(window,window[1:]):
            t=(a['stamp']+b['stamp'])/2
            bias=bias0+(bias1-bias0)*max(0.,min(1.,(t-t0)/(t1-t0)))
            angle+=((a['raw_gyro_radps']+b['raw_gyro_radps'])/2-bias)*(b['stamp']-a['stamp'])
        measured=math.degrees(angle)
        if measured*reference<=0:
            excluded.append({'start_s':first['elapsed_s'],'reason':'reference/direction mismatch',
                             'measured_deg':measured,'reference_deg':reference})
            continue
        turns.append(dict(start_s=first['elapsed_s'],end_s=last['elapsed_s'],
            reference_deg=reference,measured_deg=measured,bias_before=bias0,bias_after=bias1,
            gain=reference/measured))
    if len(turns)<2 or not any(t['reference_deg']<0 for t in turns) or not any(t['reference_deg']>0 for t in turns):
        raise ValueError('Need valid clockwise and counterclockwise reference turns')
    gain=sum(t['measured_deg']*t['reference_deg'] for t in turns)/sum(t['measured_deg']**2 for t in turns)
    if not .9<gain<1.1 or max(t['gain'] for t in turns)-min(t['gain'] for t in turns)>.015:
        raise ValueError('Inconsistent scale; repeat level reference turns')
    for turn in turns:
        turn['fitted_error_deg']=gain*turn['measured_deg']-turn['reference_deg']
    return dict(yaw_rate_scale=gain,turns=turns,excluded=excluded,
                wheel_parameters_changed=False,parameters_applied=False,
                note='Fit to operator-marked reference turns, not independent validation of future turn accuracy.')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('log',type=Path)
    args=parser.parse_args()
    with args.log.open() as source:
        rows=[json.loads(line) for line in source]
    print(json.dumps(analyze(rows),ensure_ascii=False,indent=2))
