#!/usr/bin/env python3
"""
Script to run a Docker container for Kachaka development
Usage: ./run_docker_container.py [PROJECT_NAME] [--real|-r]
"""

import argparse
import os
import platform
import subprocess
import sys
from pathlib import Path


class DockerContainerRunner:
    def __init__(self):
        # Change to the directory containing the script
        os.chdir(Path(__file__).parent)
        
        # Set up user/group IDs
        self.user_id = os.getuid()
        self.group_id = os.getgid()
        os.environ['USER_ID'] = str(self.user_id)
        os.environ['GROUP_ID'] = str(self.group_id)
        
        self.project = None
        self.container = None
        self.use_real = False
        self.ros_domain_id = 1

    def setup_project_name(self, project_name=None):
        """Set up project name and container name"""
        if os.environ.get('KACHAKA_PROJECT_NAME'):
            self.project = os.environ['KACHAKA_PROJECT_NAME']
        elif project_name:
            self.project = project_name
        else:
            try:
                self.project = input("Enter project name: ").strip()
                if not self.project:
                    print("Error: Project name cannot be empty")
                    sys.exit(1)
            except (KeyboardInterrupt, EOFError):
                print("\nOperation cancelled by user")
                sys.exit(1)

        os.environ['KACHAKA_PROJECT_NAME'] = self.project
        # Include mode info in project name to maintain separate container states
        project_with_mode = f"{self.project}_{'real' if self.use_real else 'sim'}"
        self.container = f"{project_with_mode}_kachaka_project_1"

        print(f"PROJECT={self.project}")
        print(f"MODE={'real' if self.use_real else 'sim'}")
        print(f"CONTAINER={self.container}")

    def run_command(self, cmd, shell=True, check=True, capture_output=False, env=None):
        """Run a command with proper error handling"""
        try:
            if env:
                current_env = os.environ.copy()
                current_env.update(env)
            else:
                current_env = None
            
            result = subprocess.run(
                cmd, shell=shell, check=check, 
                capture_output=capture_output, text=True,
                env=current_env
            )
            return result
        except subprocess.CalledProcessError:
            if not check:
                return None
            raise

    def container_exists(self):
        """Check if container exists"""
        try:
            result = self.run_command(
                "docker ps -a --format '{{.Names}}'", 
                capture_output=True
            )
            return result and self.container in result.stdout.split('\n')
        except:
            return False

    def run_docker_container(self):
        """Run the Docker container"""
        # Handle different OS environments
        display = os.environ.get('DISPLAY', ':0')
        profile = "linux"
        
        if platform.system() == "Darwin":
            display = ":0"
            profile = "darwin"
            print("\n" + "=" * 44)
            print("To view X11 applications from the container:")
            print("1. Open a web browser and go to: http://localhost:8080/vnc.html")
            print("2. Click 'Connect' in the browser")
            print("3. Now you should see X11 applications that you run in the container")
            print("=" * 44 + "\n")

        # Remove any conflicting containers from other modes
        self._remove_project_containers()
        
        # Check if current container already exists
        if self.container_exists():
            print(f"Container '{self.container}' already exists.")
            print("Starting the existing container...")
            try:
                self.run_command(f"docker start {self.container}")
            except subprocess.CalledProcessError:
                print("Failed to start container. Removing and recreating...")
                self.run_command(f"docker rm -f {self.container}", check=False)
                self._create_new_container(display, profile)
        else:
            print(f"Creating a new container '{self.container}'...")
            self._create_new_container(display, profile)

    def _create_new_container(self, display, profile):
        """Create a new container"""
        env = {
            'DISPLAY': display,
            'ROS_DOMAIN_ID': str(self.ros_domain_id)
        }
        # Use project name with mode for Docker Compose project name
        project_with_mode = f"{self.project}_{'real' if self.use_real else 'sim'}"
        cmd = (f"docker compose --compatibility -p {project_with_mode} "
               f"--profile {profile} -f ./docker/docker-compose.yml up -d")
        self.run_command(cmd, env=env)

    def setup_x11_auth(self):
        """Set up X11 authentication"""
        if platform.system() == "Darwin":
            print("macOS detected, X11 will be set up through NoVNC.")
            return

        display = os.environ.get('DISPLAY')
        if not display:
            return

        try:
            # Get xauth list
            result = self.run_command(
                f"xauth list {display}", 
                capture_output=True, check=False
            )
            xauth_result = result.stdout.strip() if result else ""
            print(f"XAUTH_RESULT: {xauth_result}")

            if not xauth_result:
                self.run_command(f"xauth generate {display} .", check=False)

            # Get xauth list again
            result = self.run_command(
                f"xauth list {display}", 
                capture_output=True, check=False
            )
            
            if result and result.stdout.strip():
                xauth_list = result.stdout.strip().split()
                if len(xauth_list) >= 3:
                    xauth_protocol = xauth_list[1]
                    xauth_key = xauth_list[2]
                    
                    docker_cmd = (f"docker exec -it {self.container} bash -c "
                                f"\"touch \\$HOME/.Xauthority; "
                                f"xauth add {display} {xauth_protocol} {xauth_key}\"")
                    self.run_command(docker_cmd, check=False)
                else:
                    print("Warning: Could not set up X11 authentication. GUI applications may not work.")
            else:
                print("Warning: Could not set up X11 authentication. GUI applications may not work.")
        except Exception:
            print("Warning: Could not set up X11 authentication. GUI applications may not work.")

    def enter_container(self):
        """Enter the container"""
        self.run_command(f"docker exec -it {self.container} bash", check=False)

    def parse_arguments(self):
        """Parse command line arguments"""
        parser = argparse.ArgumentParser(
            description="Script to run a Docker container for Kachaka development",
            formatter_class=argparse.RawDescriptionHelpFormatter
        )
        parser.add_argument('project_name', nargs='?', help='Project name')
        parser.add_argument('--real', '-r', action='store_true', 
                           help='Use real robot mode')
        
        args = parser.parse_args()
        
        self.use_real = args.real
        self.ros_domain_id = 0 if self.use_real else 1
        
        return args.project_name

    def run(self):
        """Main execution flow"""
        project_name = self.parse_arguments()
        
        print("Starting Docker container for Kachaka development...")
        self.setup_project_name(project_name)
        
        print("Running Docker container...")
        self.run_docker_container()
        
        print("Setting up X11 authentication...")
        self.setup_x11_auth()
        
        print("Entering the container...")
        self.enter_container()

    def _remove_project_containers(self):
        """Remove existing project containers to handle sim/real mode switching"""
        # Only remove the container that's different from current mode
        if self.use_real:
            # If using real mode, remove sim container
            container_to_remove = f"{self.project}_sim_kachaka_project_1"
        else:
            # If using sim mode, remove real container
            container_to_remove = f"{self.project}_real_kachaka_project_1"
        
        try:
            # Check if container exists
            result = self.run_command(
                f"docker ps -a --format '{{{{.Names}}}}' | grep -E '^{container_to_remove}$'",
                capture_output=True, check=False
            )
            if result and result.stdout.strip():
                print(f"Removing existing container '{container_to_remove}'...")
                self.run_command(f"docker rm -f {container_to_remove}", check=False)
        except:
            pass  # Ignore errors if container doesn't exist


if __name__ == "__main__":
    runner = DockerContainerRunner()
    runner.run()
