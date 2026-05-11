# Industrial Mobile Robotics Lab - Workspace

## Workspace Structure

```
imrl_workspace/
├── fleet_management/   # Fleet management simulation and tasks
├── motion_control/     # Motion control simulation and tasks
├── json_examples/      # Example JSON messages (order, state, pose, etc.)
├── environment.yml     # Conda environment definition
└── README.md
```

The practical part of this course is organized into two main folders: `fleet_management` and `motion_control`. Each of these contains a `data` folder and a `src` folder. The `data` folders contain the data required to run the simulations and the `src` folders contain the source code. The `json_examples` folder provides examples of the JSON files you will work with.

Detailed, topic-specific instructions are in the respective sub-READMEs:
- [fleet_management/README.md](fleet_management/README.md)
- [motion_control/README.md](motion_control/README.md)

---

## Getting Started

### Supported Environments

The simulations require a Linux environment. Three options are supported:

| Option | Notes |
|--------|-------|
| **Native Ubuntu 24.04** | Full performance, no extra setup |
| **Dual-boot Ubuntu 24.04** | Same as native once booted into Ubuntu |
| **WSL2 (Windows Subsystem for Linux)** | Works on Windows 10/11 — see WSL notes below |

> **WSL setup notes:**
> 1. Open PowerShell as Administrator and run `wsl --install -d Ubuntu-24.04`, then restart.
> 2. **GUI support (required for the simulation window):**
>    - **Windows 11**: WSLg is built in — no extra setup needed.
>    - **Windows 10**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (launch with *Disable access control* checked), then add `export DISPLAY=:0` to your `~/.bashrc`.
> 3. All remaining steps below are run **inside the WSL terminal**, not in PowerShell.

---

### Step 1 — Install VS Code

Install [VS Code](https://code.visualstudio.com/Download).

- **Native/dual-boot:** follow the Ubuntu instructions on the download page.
- **WSL:** install VS Code on the **Windows** side. The WSL integration (remote connection into Linux) is handled automatically when you run `code .` from a WSL terminal for the first time.

---

### Step 2 — Install Git

```bash
sudo apt update && sudo apt install -y git
```

Configure your identity (required for commits):

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

Verify:

```bash
git --version
```

---

### Step 3 — Clone the Repository

```bash
git clone <repository-url>
cd imrl_workspace
```

> **WSL users:** run this inside the WSL terminal so the files live in the Linux filesystem — this avoids performance and permission issues that occur when working on Windows-side paths (`/mnt/c/...`) from WSL.

---

### Step 4 — Open the Workspace in VS Code

From the terminal inside the `imrl_workspace` folder:

```bash
code .
```

- **WSL:** VS Code will automatically install the WSL extension and reopen in the WSL environment on first use.
- Install the **Python** extension in VS Code if prompted (or search for it in the Extensions sidebar).

---

### Step 5 — Set Up the Conda Environment

1. Install [Miniconda](https://www.anaconda.com/docs/getting-started/miniconda/install) if not already installed (follow the Linux instructions).
   A [conda cheatsheet](https://docs.conda.io/projects/conda/en/latest/_downloads/843d9e0198f2a193a3484886fa28163c/conda-cheatsheet.pdf) is available for reference.

2. Open a terminal in VS Code (`` Ctrl+` ``) and verify the installation:
    ```bash
    conda --version
    ```
    If `conda` is not found, initialize it for your shell, then restart the terminal:
    ```bash
    conda init bash
    ```

3. Create the virtual environment from the provided `environment.yml`:
    ```bash
    conda env create -n imrl_env -f environment.yml
    ```

4. Activate the environment:
    ```bash
    conda activate imrl_env
    ```

5. Select `imrl_env` as the Python interpreter in VS Code:
   Press `Ctrl+Shift+P`, type **Python: Select Interpreter**, and choose the entry that contains `imrl_env`.
   This ensures the run button and the Testing sidebar use the correct Python.

---

### Step 6 — Install and Verify the Mosquitto MQTT Broker

1. Install Mosquitto:
    ```bash
    sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
    sudo apt update
    sudo apt install mosquitto mosquitto-clients
    ```
    On Ubuntu, this automatically starts the broker as a systemd service on port 1883 and enables it at boot.

2. Verify the service is running:
    ```bash
    sudo systemctl status mosquitto
    ```

3. Test the broker — open two terminals and run:
    ```bash
    # Terminal 1 — subscribe
    mosquitto_sub -h localhost -t test/topic

    # Terminal 2 — publish
    mosquitto_pub -h localhost -t test/topic -m "Hello, World!"
    ```
    You should see `Hello, World!` appear in Terminal 1.
