"""Build independent A3.0 with-ID assets; preserve existing calibration on reruns."""
import argparse
import yaml
import prepare_hand_v3_0 as builder


def configure():
    builder.SOURCE = builder.ROOT / "TsenseI_A3.0_hand_withid"
    builder.SOURCE_URDF = builder.SOURCE / "urdf/A3-2691-1053yw1.urdf"
    builder.DEST = builder.ROOT / "assets/hands/TsenseI_hand_v3_0_withid"
    builder.MODEL_ID = "TsenseI_v3_0_withid"
    builder.MODEL_SUFFIX = "v3_0_withid"
    builder.DISPLAY_NAME = "TsenseI A3.0 with ID"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sync-right-keypoints", action="store_true")
    args = parser.parse_args()
    configure()
    if args.sync_right_keypoints:
        builder.sync_right_keypoints()
    else:
        builder.source_fingerprint()
        builder.archive_if_source_changed()
        fresh = not (builder.DEST / "hand_profile.yaml").exists()
        builder.prepare_urdfs()
        builder.prepare_configs()
        if fresh:
            configure_profile()
        from generate_profile_collision import generate_profile
        generate_profile(builder.MODEL_ID, profile_root=builder.DEST)
    print(builder.DEST)


def configure_profile():
    path = builder.DEST / "hand_profile.yaml"
    profile = yaml.safe_load(path.read_text(encoding="utf-8"))
    original = yaml.safe_load((builder.ROOT / "assets/hands/TsenseI_hand_v3_0/hand_profile.yaml").read_text(encoding="utf-8"))
    profile["retarget"] = original["retarget"]
    profile["collision_scope"] = "different_fingers_only"
    profile["notes"] = [
        "Independent with-ID shell geometry and 21-point annotations; not the bare A3.0 package.",
        "Canonical joint frames/axes/limits match A3.0; shared nine-linear/eleven-rotary hardware conversion.",
        "New geometry seeds require manual landmark calibration; do not inherit bare A3.0 annotations.",
        "15 conservative finger outer meshes; different-finger pairs only. Palm, wrist and mounting shell excluded.",
        "Right hand is an XZ mirror, not an independently measured right CAD export.",
    ]
    builder.dump_yaml(path, profile)


if __name__ == "__main__":
    main()
