import adsk.core, adsk.fusion, traceback, csv, math

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        
        # 1. Ask user for Max Angle input
        input_tuple = ui.inputBox("Enter maximum servo sweep angle (e.g. 30):", "Sweep Range", "30")
        if not input_tuple[0] or input_tuple[1]: # User cancelled or empty
            return
        
        try:
            max_deg = abs(int(input_tuple[0]))
        except ValueError:
            ui.messageBox("Please enter a valid integer for the angle.")
            return

        # 2. Locate target joints in model
        servo_joint = None
        wheel_joint_l = None
        wheel_joint_r = None
        
        target_names = {
            'ServoJoint': None,
            'WheelJointL': None,
            'WheelJointR': None
        }
        
        # Search all joints across components
        for j in list(root.allJoints) + list(root.allAsBuiltJoints):
            if j.name in target_names:
                target_names[j.name] = j

        servo_joint = target_names['ServoJoint']
        wheel_joint_l = target_names['WheelJointL']
        wheel_joint_r = target_names['WheelJointR']

        if not servo_joint or not wheel_joint_l or not wheel_joint_r:
            missing = [k for k, v in target_names.items() if v is None]
            ui.messageBox(f"Error: Missing joint(s): {missing}\n\nMake sure your browser tree has joints named: ServoJoint, WheelJointL, WheelJointR")
            return

        # Motion object references
        servo_motion = servo_joint.jointMotion
        wheel_l_motion = wheel_joint_l.jointMotion
        wheel_r_motion = wheel_joint_r.jointMotion
        
        desktop_path = "C:/Users/Public/ackermann_data.csv"
        
        with open(desktop_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Servo_Angle_Deg', 'Servo_Slide_mm', 'Left_Wheel_Deg', 'Right_Wheel_Deg'])
            
            # Sweep servo from -max_deg to +max_deg
            for deg in range(-max_deg, max_deg + 1, 1):
                rad = math.radians(deg)
                
                # Apply rotation to Servo Pin-Slot
                servo_motion.rotationValue = rad
                app.activeViewport.refresh()
                adsk.doEvents()
                
                # Read outputs: convert Fusion internal length (cm) to mm
                slide_mm = servo_motion.slideValue * 10.0 if hasattr(servo_motion, 'slideValue') else 0.0
                left_deg = math.degrees(wheel_l_motion.rotationValue)
                right_deg = math.degrees(wheel_r_motion.rotationValue)
                
                writer.writerow([deg, round(slide_mm, 4), round(left_deg, 4), round(right_deg, 4)])
                
        ui.messageBox(f"Export Complete!\nSaved data from -{max_deg}° to +{max_deg}° to:\n{desktop_path}")

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))