"""Targeted known-map shortcuts, budget fallback and clearance equivalence.

No camera/ToF acquisitions or robot movement. CAD motion and speed allowances
are modeled; actual tyre slip/steering latency remain physical limitations.
"""
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_connector_servo_resume import block
from check_later_laps import later_lap_fixture


def main():
    dest = ROOT/'local_workspace/inner_lane_20261027'
    dest.mkdir(parents=True, exist_ok=True)
    fixture = later_lap_fixture().split('int main()')[0]
    fixture = fixture.replace('uint32_t micros(){return 0;}',
        'uint32_t hostClock=0,hostTick=0;uint32_t micros(){uint32_t t=hostClock;hostClock+=hostTick;return t;}')
    historical = subprocess.check_output(
        ['git', 'show', '94b2e0d:src/obstacle_path.cpp'], text=True)
    legacy = block(historical, historical.index('bool laterLapRouteSafe('))
    legacy = legacy.replace('bool laterLapRouteSafe(', 'bool exhaustiveRouteSafe(')
    fixture += legacy + r'''
float absoluteTurn(const PathPoint *r){float length,peak,turn;laterLapBending(r,length,peak,&turn);return turn;}
int main(){
 // Frozen exhaustive wall-per-pillar implementation must agree with the new
 // wall-once implementation on accepted minima and every reject decision.
 int equivalence=0;
 for(int direction:{-1,1})for(int variant=0;variant<56;++variant){
  routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=(i+variant)%5==0;seats[i].injected=seats[i].confirmed;seats[i].red=i%2;}
  learned();
  for(int offset:{-80,0,80}){
   memcpy(optimizedPath,livePath,sizeof(PathPoint)*pathLength);
   for(int i=15;i<pathLength-15;++i)optimizedPath[i].x+=offset;
   float a,b,c,d;bool old=exhaustiveRouteSafe(optimizedPath,&a,&b),now=laterLapRouteSafe(optimizedPath,&c,&d);
   ++equivalence;if(old!=now||(old&&(fabsf(a-c)>.001f||fabsf(b-d)>.001f)))return 10;
  }
 }
 std::cout<<"EQUIVALENCE "<<equivalence<<" PASS\n";
 // Deliberately exhaust the optional budget, including across micros wrap.
 int budgets=0;
 for(uint32_t clock:{0UL,0xffffe000UL})for(uint32_t tick:{1000UL,30000UL}){
  routeTurnSign=-1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  learned();hostTick=0;buildOptimizedPath();
  PathPoint prior[OBSTACLE_MAX_PATH_WAYPOINTS],rollback[OBSTACLE_MAX_PATH_WAYPOINTS];
  memcpy(prior,optimizedPath,sizeof(PathPoint)*pathLength);
  hostClock=clock;hostTick=tick;shortenKnownInnerRoute(optimizedPath,rollback);hostTick=0;
  for(int i=0;i<pathLength;++i)if(optimizedPath[i].x!=prior[i].x||optimizedPath[i].y!=prior[i].y)return 11;
  ++budgets;
 }
 std::cout<<"BUDGET "<<budgets<<" PASS; checked shape retained, timer wrap\n";
 int direction,id=0,colors[24];
 while(std::cin>>direction){
  for(int &c:colors)if(!(std::cin>>c))return 3;
  routeTurnSign=direction;resetState();baseline();hostTick=0;
  for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=colors[i]!=0;seats[i].red=colors[i]==2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  if(!buildOptimizedPath())return 12;
  float length,peak,turn,wall,pillar;
  float bend=laterLapBending(optimizedPath,length,peak,&turn),seconds=laterLapTargetTime(optimizedPath);
  if(!laterLapRouteSafe(optimizedPath,&wall,&pillar))return 13;
  for(int i=0;i<pathLength;++i)std::cout<<"POINT "<<id<<" "<<i<<" "<<optimizedPath[i].x<<" "<<optimizedPath[i].y<<"\n";
  for(int i=0;i<24;++i)if(seats[i].confirmed)std::cout<<"SIGN "<<id<<" "<<seats[i].x<<" "<<seats[i].y<<" "<<seats[i].red<<"\n";
  float trackedWall,trackedPillar;bool ok=driveTwoLaps(0,trackedWall,trackedPillar);
  if(!ok||trackedWall<0||trackedPillar<30)return 14;
  std::cout<<"INNER "<<id++<<" "<<length<<" "<<seconds<<" "<<bend<<" "<<turn<<" "<<wall<<" "<<pillar
   <<" "<<trackedWall<<" "<<trackedPillar<<"\n";
 }
}
'''
    cases = []
    # CW outer GREEN at the end, next outer RED at entry (user's example),
    # followed by empty or solitary-middle next sections, plus CCW mirrors.
    for direction in (-1, 1):
        outer = 1 if direction < 0 else 0
        for mode in range(4):
            colors = [0]*24
            if mode:
                colors[10+outer] = 1 if direction < 0 else 2
            if mode == 1:
                colors[12+outer] = 2 if direction < 0 else 1
            if mode == 2:
                colors[14+outer] = 2 if direction < 0 else 1
            cases.append(dict(direction=direction, mode=mode, colors=colors))
    inputs = '\n'.join(' '.join(map(str, [c['direction'], *c['colors']])) for c in cases)+'\n'
    results = {}
    for name in ('before', 'after'):
        code = fixture
        if name == 'before':
            code = code.replace('        shortenKnownInnerRoute(optimizedPath,beforeLaneCarry);',
                                '        /* optional inner shortcut disabled */')
        cpp, exe = dest/f'targeted_{name}.cpp', dest/f'targeted_{name}.exe'
        cpp.write_text(code)
        build = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
            '-I', str(ROOT/'include'), '-I', str(ROOT/'local_workspace/later-laps'),
            str(cpp), '-o', str(exe)], text=True, capture_output=True)
        (dest/f'targeted_{name}_build.txt').write_text(build.stdout+build.stderr)
        if build.returncode:
            print('\n'.join(build.stderr.splitlines()[:30]));raise SystemExit(build.returncode)
        run = subprocess.run([str(exe)], input=inputs, text=True, capture_output=True)
        (dest/f'targeted_{name}_output.txt').write_text(run.stdout+run.stderr)
        if run.returncode:
            print(run.stdout[-1500:]);raise SystemExit(run.returncode)
        rows, points, signs = [], [[] for _ in cases], [[] for _ in cases]
        for line in run.stdout.splitlines():
            if line.startswith(('EQUIVALENCE', 'BUDGET')):
                print(name, line)
            elif line.startswith('INNER '):
                f = line.split();rows.append(dict(zip(
                    ('length_mm', 'target_s', 'bending', 'absolute_turn_rad', 'wall_mm',
                     'pillar_mm', 'tracked_wall_mm', 'tracked_pillar_mm'), map(float, f[2:]))))
            elif line.startswith('POINT '):
                f = line.split();points[int(f[1])].append(list(map(float, f[3:])))
            elif line.startswith('SIGN '):
                f = line.split();signs[int(f[1])].append(list(map(float, f[2:])))
        assert len(rows) == len(cases)
        results[name] = dict(metrics=rows, points=points, signs=signs)
    for i, c in enumerate(cases):
        a, b = results['before']['metrics'][i], results['after']['metrics'][i]
        assert b['length_mm'] <= a['length_mm']+.05
        assert b['target_s'] <= a['target_s']+.001
        inner_colour = 2 if c['direction'] < 0 else 1
        continuous_inner = all(colour in (0, inner_colour) for colour in c['colors'][6:])
        # Only a continuous inner corridor may use the new small connection;
        # mixed bypasses retain the original tight absolute-turn allowance.
        extra_turn = .201 if continuous_inner else .011
        assert b['absolute_turn_rad'] <= a['absolute_turn_rad']+extra_turn
        if c['mode'] == 0:
            assert b['absolute_turn_rad'] > a['absolute_turn_rad']+.011
        print('TARGET', c['direction'], c['mode'], 'saved_mm', round(a['length_mm']-b['length_mm'], 2),
              'target_s_saved', round(a['target_s']-b['target_s'], 3))
    for i in (2, 6):
        assert results['after']['metrics'][i]['length_mm'] < results['before']['metrics'][i]['length_mm']-50
    (dest/'targeted.json').write_text(json.dumps(dict(cases=cases, **results), indent=2)+'\n')
    # Portable standalone comparison, generated and reproducible from this script.
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1300 680">',
           '<rect width="1300" height="680" fill="white"/>']
    for panel, case in enumerate((2, 6)):
        x0 = panel*650+30
        svg.append(f'<text x="{x0}" y="30" font-family="sans-serif" font-size="18">'
                   f'{"CW" if cases[case]["direction"]<0 else "CCW"}: outer sign to next middle sign</text>')
        svg.append(f'<g transform="translate({x0+300} 350) scale(.18 -.18)">'
                   '<rect x="-1500" y="-1500" width="3000" height="3000" fill="#f5f5f5" stroke="#333" stroke-width="12"/>'
                   '<rect x="-500" y="-500" width="1000" height="1000" fill="#ddd" stroke="#333" stroke-width="12"/>')
        for name, color in (('before', '#999'), ('after', '#1265c7')):
            pts = ' '.join(f'{x},{y}' for x,y in results[name]['points'][case])
            svg.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="14"/>')
        for x,y,red in results['after']['signs'][case]:
            svg.append(f'<circle cx="{x}" cy="{y}" r="42.5" fill="none" stroke="#333" stroke-width="4"/>'
                       f'<rect x="{x-25}" y="{y-25}" width="50" height="50" fill="{"#cf3030" if red else "#169544"}"/>')
        svg.append('</g>')
    svg.append('<text x="40" y="660" font-family="sans-serif" font-size="16">Gray: checked prior route. Blue: accepted inner shortcut. CAD model; no measured field trajectory.</text></svg>')
    (dest/'targeted_routes.svg').write_text('\n'.join(svg))
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    img = Image.new('RGB', (1300, 680), 'white');draw = ImageDraw.Draw(img)
    for panel, case in enumerate((2, 6)):
        x0 = panel*650+330
        xy = lambda p: (x0+.18*p[0], 350-.18*p[1])
        draw.text((panel*650+30, 20), 'CW' if cases[case]['direction']<0 else 'CCW', fill='black')
        draw.rectangle([xy((-1500,1500)), xy((1500,-1500))], fill='#f5f5f5', outline='black', width=2)
        draw.rectangle([xy((-500,500)), xy((500,-500))], fill='#ddd', outline='black', width=2)
        for name, color in (('before', '#999'), ('after', '#1265c7')):
            draw.line([xy(p) for p in results[name]['points'][case]], fill=color, width=3)
        for x,y,red in results['after']['signs'][case]:
            px,py = xy((x,y));draw.ellipse((px-8,py-8,px+8,py+8), outline='black')
            draw.rectangle((px-4,py-4,px+4,py+4), fill='#cf3030' if red else '#169544')
    draw.text((30,650), 'Gray: previous route. Blue: accepted shortcut. Synthetic CAD replay.', fill='black')
    img.save(dest/'targeted_routes.png')


if __name__ == '__main__':
    main()
