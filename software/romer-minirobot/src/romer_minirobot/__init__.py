from .utils import is_running_on_pico
if not is_running_on_pico():
    from .robot import MiniRobot
from . import urtps
from . import modules

def clone_github_repo(repo_url, temp_dir):
    import subprocess
    try:
        # Clone the GitHub repository to the temporary directory
        command = f"git clone {repo_url} {temp_dir}"
        
        # Execute the command
        process = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        print(f"Successfully cloned the repository: {repo_url}")
        print(process.stdout)
        
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while cloning the repository: {repo_url}")
        print(e.stderr)

def transfer_to_pico(src_dir, device_path):
    import os
    import subprocess
    try:
        # Start rshell and connect to the Pico
        rshell_command = f"rshell -p {device_path}"
        process = subprocess.Popen(rshell_command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Wait for rshell to start
        process.stdin.write("connect serial\n")
        process.stdin.flush()

        # Transfer directory or file
        for root, dirs, files in os.walk(src_dir):
            for file in files:
                full_file_path = os.path.join(root, file)
                relative_path = os.path.relpath(full_file_path, src_dir)
                pico_path = f"/pyboard/{relative_path}"
                
                command = f"cp {full_file_path} {pico_path}\n"
                process.stdin.write(command)
                process.stdin.flush()

        # Close rshell
        process.stdin.write("exit\n")
        process.stdin.flush()

        stdout, stderr = process.communicate()
        
        if process.returncode != 0:
            print(f"Error transferring files: {stderr}")
        else:
            print(f"Successfully transferred files from '{src_dir}' to the Pico.")

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    import argparse
    import shutil
    import tempfile
    parser = argparse.ArgumentParser(description="Transfer files or folders to Raspberry Pi Pico and optionally install a GitHub repository")
    parser.add_argument("file_or_folder", help="Path to the file or folder to transfer")
    parser.add_argument("device_path", help="Path to the device (e.g., /dev/ttyACM0)")
    parser.add_argument("--picoinstall", help="Install and transfer a specific GitHub repository to Pico", action="store_true")
    parser.add_argument("--folder", help="Specific folder within the repository to transfer", default=None)
    
    args = parser.parse_args()

    repo_url = "https://github.com/SalihTasdelen/romer-minirobot.git"  # Replace with the actual repository URL

    if args.picoinstall:
        # Create a temporary directory for the GitHub repository
        temp_dir = tempfile.mkdtemp()
        
        # Clone the GitHub repository to the temporary directory
        clone_github_repo(repo_url, temp_dir)
        
        # Define the source directory for transfer

        src_dir = temp_dir + "/src/romer_minirobot"

        # Check if the specified folder exists
        # if not os.path.exists(src_dir):
        #     print(f"Error: The specified folder '{src_dir}' does not exist in the repository.")
        #     shutil.rmtree(temp_dir)
        #     return
        
        # Transfer the folder to the Pico
        transfer_to_pico(src_dir, args.device_path)
        
        # Clean up the temporary directory
        shutil.rmtree(temp_dir)
        
    else:
        # Transfer the specified file or folder to the Pico
        transfer_to_pico(args.file_or_folder, args.device_path)
