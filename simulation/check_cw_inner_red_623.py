"""CW outer GREEN -> inner RED: later-lap lane and CAD tracked clearance."""
import re
from check_later_laps import later_lap_fixture
from check_connector_servo_resume import ROOT, block
from check_returned_603_and_parking_practice import run


def main():
    source=(ROOT/'src/obstacle_path.cpp').read_text()
    rule=block(source,source.index('    if(routeTurnSign<0 && sectionLayoutMode==OBSTACLE_SECTION_LAYOUT_OFFICIAL &&',
                                 source.index('float optimizedClearanceForSeat(')))
    fixture=later_lap_fixture().split('int main()')[0]
    fixture=fixture.replace('int failReason=0,', 'float redWall=1e9f; int failReason=0,')
    fixture=fixture.replace('minimumWall=fminf(minimumWall,sample.wallMm);',
        'minimumWall=fminf(minimumWall,sample.wallMm);if(x<-550 && y>100 && y<650)redWall=fminf(redWall,sample.wallMm);')
    main=r'''
int main(){
 routeTurnSign=-1;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
 for(int i:{0,4,7,10,15,19,22}){seats[i].confirmed=seats[i].injected=true;seats[i].red=i==4||i==10||i==19;}
 if(validatedClearanceForSeat(10)!=260)return 1;
 const float clearance=optimizedClearanceForSeat(10);
 learned();joinSameColourStraightEnds(livePath,true);float wall,pillar;
 if(!driveTwoLaps(0,wall,pillar))return 2;
 float lane=0;for(int i=0;i<pathLength;++i)
  if(fabsf(baselinePath[i].distanceMm-seats[10].pathDistanceMm)<1)lane=optimizedPath[i].x;
 std::cout<<"RED METRICS "<<clearance<<" "<<lane<<" "<<redWall<<" "<<pillar<<"\n";
 // Controls: CW inner RED without preceding outer GREEN and every CCW
 //case retain the old clearance; first-lap clearance never changes.
 seats[7].confirmed=false;if(optimizedClearanceForSeat(10)!=210)return 3;
 routeTurnSign=1;if(optimizedClearanceForSeat(10)!=210)return 4;
}
'''
    metrics=[]
    for name,code in [('old',fixture.replace(rule,'')),('new',fixture)]:
        run('623-red-'+name,code+main)
        output=(ROOT/f'local_workspace/returned-603/623-red-{name}/output.txt').read_text()
        metrics.append(tuple(map(float,re.search(r'RED METRICS ([\d.e+-]+) ([\d.e+-]+) ([\d.e+-]+) ([\d.e+-]+)',output).groups())))
    old,new=metrics
    assert new[0]==180 and old[0]==210
    assert new[1]<old[1]-20
    assert new[2]>old[2]+10 and new[3]>=30
    print('623 CW RED PASS: outward lane shift and increased modeled wall reserve')


if __name__=='__main__':
    main()
