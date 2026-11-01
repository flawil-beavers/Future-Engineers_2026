"""628 anchors, measured30mm rear: normal bidirectional parking without extra10."""
from check_final_parking_sequence import ROOT, fixture_source
from check_returned_603_and_parking_practice import run


def main():
    script=(ROOT/'simulation/check_parking_adaptive_completion.py').read_text()
    code=script.split("source=fixture_source()+r'''",1)[1].split("'''",1)[0]
    code=code.replace('for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})',
                      'for(float gap:{253.3f})')
    code=code.replace('for(float scale:{.95f,1.f,1.05f,1.10f,1.15f})',
                      'for(float scale:{1.134f,1.178f,1.218f})')
    begin=code.index(' for(auto anchor:');end=code.index('){',begin)+2
    code=code[:begin]+''' for(auto anchor:{PositionEstimate{269.1f,295.8f,-.3f},
         PositionEstimate{64.2f,126.5f,35.5f}}){'''+code[end:]
    code=code.replace('state=FP_REPLAN_BRAKE;stateStartMs=0;',
        'if(anchor.y_mm<250){entryRadiusScale=1.178f;entryRadiusMeasured=true;}\n  state=FP_REPLAN_BRAKE;stateStartMs=0;')
    code=code.replace('++total;bool ok=',
        'if(robot_logger.lines.str().find("Checked10mm rearward escape")!=std::string::npos)return 6;\n  ++total;bool ok=')
    run('628-balanced',fixture_source()+code)


if __name__=='__main__':
    main()
