"""
=============================================================================
  VISION-BASED OUTDOOR NAVIGATION FOR UNMANNED GROUND VEHICLES (UGV)
  Interactive Project Launcher (2nd Year Engineering Mini-Project)
=============================================================================
This launcher provides a simple menu interface for students and evaluators
to run and test all components of the project with a single command:
    python run_project.py
"""
import os
import subprocess
import sys


def print_banner():
    print("=" * 75)
    print("   VISION-BASED OUTDOOR NAVIGATION FOR UNMANNED GROUND VEHICLES (UGV)")
    print("   2nd Year Engineering Academic Mini-Project Prototype")
    print("   Tech Stack: Python 3 | OpenCV | NumPy | Matplotlib")
    print("=" * 75)


def run_command(cmd_list, description):
    print(f"\n>>> Running: {description}")
    print(f">>> Command: {' '.join(cmd_list)}\n")
    try:
        subprocess.run(cmd_list, check=True)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception as e:
        print(f"\n[ERROR] Execution failed: {e}")
    input("\nPress Enter to return to main menu...")


def menu():
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print_banner()
        print("\nChoose an option to demonstrate:")
        print("  [1] Launch Web Dashboard & Multi-Module Studio (Recommended)")
        print("  [2] Live Vision Navigation in OpenCV Window (Desktop)")
        print("  [3] Closed-Loop 4WD Skid-Steer Autonomous Simulation")
        print("  [4] Test Scenario A: Sunny Curved Trail")
        print("  [5] Test Scenario B: Shadowed Gravel Path (Shadow Test)")
        print("  [6] Test Scenario C: Sharp Switchback Turns")
        print("  [7] Generate Performance Evaluation Report & Graphs")
        print("  [8] Open Student Project Report & Viva Guide")
        print("  [0] Exit")
        print("-" * 75)

        choice = input("Enter choice [0-8]: ").strip()

        if choice == "1":
            run_command([sys.executable, "serve_website.py"],
                        "Web Mission Control Dashboard & Multi-Module Studio")
        elif choice == "2":
            run_command([sys.executable, "main.py", "--source", "outdoor_video.mp4"],
                        "Main Vision-Based Navigation Prototype (OpenCV)")
        elif choice == "3":
            run_command([sys.executable, "simulate_ugv.py"],
                        "Closed-Loop 4WD Skid-Steer Rover Simulator")
        elif choice == "4":
            run_command([sys.executable, "main.py", "--source", "test_clips/sunny_curved_trail.mp4"],
                        "Scenario 1: Sunny Curved Trail")
        elif choice == "5":
            run_command([sys.executable, "main.py", "--source", "test_clips/shadowed_gravel_path.mp4"],
                        "Scenario 2: Shadowed Gravel Path")
        elif choice == "6":
            run_command([sys.executable, "main.py", "--source", "test_clips/sharp_switchback.mp4"],
                        "Scenario 3: Sharp Switchback Path")
        elif choice == "7":
            run_command([sys.executable, "evaluate_runs.py"],
                        "Generate Evaluation Metrics & 4-Panel Plots")
        elif choice == "8":
            report_path = os.path.abspath("STUDENT_PROJECT_REPORT.md")
            print(f"\nOpening Student Project Report: {report_path}")
            if os.name == "nt":
                os.system(f'start "" "{report_path}"')
            else:
                os.system(f'xdg-open "{report_path}"')
            input("\nPress Enter to return to main menu...")
        elif choice == "0":
            print("\nExiting. Good luck with your project presentation!\n")
            break
        else:
            print("\n[!] Invalid choice. Please enter a number between 0 and 8.")
            input("Press Enter to continue...")


if __name__ == "__main__":
    menu()
