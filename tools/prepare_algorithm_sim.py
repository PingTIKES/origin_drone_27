#!/usr/bin/env python3
"""Apply narrowly checked PX4 v1.14.2 SITL clock/EV and OpenVINS startup patches.
Run on the Ubuntu source workspaces, then rebuild both dependencies.
Hardware PX4 sources are NOT a target of this tool.
"""
import argparse
from pathlib import Path
import subprocess


def write_checked(path,old,new,marker):
    text=path.read_text(encoding='utf-8')
    if marker in text:return
    if text.count(old)!=1:raise RuntimeError(f'unsupported source version/anchor: {path}')
    backup=path.with_name(path.name+'.rm27-backup')
    if not backup.exists():backup.write_text(text,encoding='utf-8')
    path.write_text(text.replace(old,new),encoding='utf-8',newline='\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4',required=True)
    p.add_argument('--openvins',required=True)
    a=p.parse_args()
    px4=Path(a.px4).resolve()
    tag=subprocess.check_output(['git','describe','--tags','--exact-match','HEAD'],cwd=px4,text=True).strip()
    if tag!='v1.14.2':raise RuntimeError('requires PX4 v1.14.2 source, found '+tag)
    source=px4/'src/modules/uxrce_dds_client/uxrce_dds_client.cpp'
    old='\t// latest round trip time (RTT)'
    new='''#if defined(__PX4_POSIX)
    // RM27_SIM_CLOCK: /clock is the clock on BOTH sides in algorithm SITL.
    if (getenv("RM27_SIM_CLOCK") && strcmp(getenv("RM27_SIM_CLOCK"), "1") == 0) {
        session->time_offset = 0;
        return;
    }
#endif
'''+old
    write_checked(source,old,new,'// RM27_SIM_CLOCK:')
    rc=px4/'ROMFS/px4fmu_common/init.d-posix/px4-rc.params'
    text=rc.read_text(encoding='utf-8')
    marker='# RM27_VISION_ONLY'
    if marker not in text:
        backup=rc.with_name(rc.name+'.rm27-backup')
        if not backup.exists():backup.write_text(text,encoding='utf-8')
        text+='''
# RM27_VISION_ONLY: applies only when explicitly started by start_algorithm_sim.sh.
if [ "$RM27_SIM_CLOCK" = "1" ]; then
    param set EKF2_GPS_CTRL 0
    param set EKF2_EV_CTRL 15
    param set EKF2_HGT_REF 3
    param set EKF2_MAG_TYPE 5
    param set EKF2_EV_POS_X 0
    param set EKF2_EV_POS_Y 0
    param set EKF2_EV_POS_Z 0
    param set MPC_XY_VEL_MAX 0.5
    param set MPC_XY_CRUISE 0.5
    param set COM_OF_LOSS_T 0.5
    param set COM_OBL_RC_ACT 4
    param set COM_POSCTL_NAVL 1
fi
'''
        rc.write_text(text,encoding='utf-8',newline='\n')
    header=Path(a.openvins)/'ov_msckf/src/core/VioManager.h'
    old='bool initialized() { return is_initialized_vio && timelastupdate != -1; }'
    new='bool initialized() { return is_initialized_vio; } // RM27_STATIC_VIO: publish after successful static initialization'
    write_checked(header,old,new,'RM27_STATIC_VIO')
    print('Patched source with .rm27-backup copies. Rebuild PX4 SITL and OpenVINS before running.')


if __name__=='__main__':main()
