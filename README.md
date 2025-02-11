# AS-Fuzzer++

AS-Fuzzer++ is an advanced fuzzing framework designed to **efficiently generate critical test scenarios** for **Autonomous Driving Systems (ADS)** in **complex road environments**. It extends the capabilities of **AS-Fuzzer** by incorporating **scenario segmentation, co-evolutionary genetic algorithms (CEGA), dynamic obstruction handling, and congestion-aware vehicle rerouting**, significantly improving **testing efficiency, scenario diversity, and defect detection rates**.

---

## **Overview**
AS-Fuzzer++ introduces several key enhancements over AS-Fuzzer:

- **Scenario Segmentation & Parallel Execution**:  
  - Large-scale scenarios are **divided into atomic sub-scenarios**, enabling **parallel processing** and **higher interaction rates**.
  - Supports **more diverse and critical test cases** while maintaining computational efficiency.

- **Co-evolutionary Genetic Algorithm (CEGA) for Optimized Scenario Generation**:  
  - Introduces **co-evolutionary mechanisms** to **generate diverse, high-risk scenarios**.
  - Enhances **interaction rates** and **exposes critical ADS failures** more effectively.

- **Block Solver Mechanism**:  
  - Dynamically **resolves stuck scenarios** where ADS vehicles are blocked due to external factors.
  - Ensures **uninterrupted testing cycles**, reducing failed test cases.

- **Congestion-aware Vehicle Rerouting**:  
  - Adjusts **traffic participant behaviors dynamically**, ensuring that NPC vehicles **engage meaningfully** with the ADS.
  - **Prevents low-interaction scenarios** and increases **realism in urban congestion testing**.

---

### **Workflow**
![overview](doc/overview.pdf)

1. **Genetic Algorithm Library (GA-Lib)**:  
   - Maintains **atomic test scenarios** derived from **large composite scenarios**.
   - Each scenario evolves **independently**, generating **parameterized local test cases** that require simulation and evaluation.

2. **Local Scenario Selector**:  
   - Defines **critical road facilities** for **ADS operation**.  
   - Dynamically **retrieves scenarios from GA-Lib**, sending them to the **Scenario Executor** for simulation and fitness evaluation.

3. **Simulation Executor**:  
   - **Facilitates communication between the ADS and simulator**.  
   - Initializes **ADS test cases**, collects route-planning outputs, **segments large scenarios into localized test cases**, and **sends scenario features** to the Local Scenario Selector.

4. **Block Solver Mechanism**:  
   - **Detects and resolves ADS blockages** caused by environmental factors.  
   - Ensures that **testing continues seamlessly** without unnecessary failures.

5. **Unsafe Behavior Monitor & Evaluation**:  
   - Detects **collisions, unsafe behaviors, or minor violations**.  
   - Logs **critical ADS failures** and **feeds evaluation results back into the GA pipeline** for scenario optimization.

---

## **Usage**
### **1. Before You Start**
Ensure the following dependencies are installed:

- **Nvidia Docker** (for GPU acceleration) ? [Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- **Carla 0.9.14** (Simulation Environment)
- **Carla Apollo Bridge** (for ADS-Testing)  
  Follow the official installation guide: [Carla Apollo Bridge](https://github.com/guardstrikelab/carla_apollo_bridge/blob/master/docs/GettingStarted.md).

We have **customized the original `carla_apollo_bridge` repository** with **remote control scripts for Apollo**.  
**Please use our forked version**:

```bash
# Using SSH
git clone git@github.com:cfs4819/carla_apollo_bridge.git

# Using HTTPS
git clone https://github.com/cfs4819/carla_apollo_bridge.git


### 2. Clone this repository and install dependencies
```bash
cd path_to_apollo/modules
git clone https://github.com/cfs4819/as_fuzz.git
```

Install dependencies in apollo container
```bash
cd path_to_apollo
./docker/script/dev_into.sh
# inside apollo container
cd /apollo/modules/as_fuzz & pip install -r requirement.txt
```

### 3. Running

```bash
# inside apollo container
cd /apollo/modules/as_fuzz
python asfuzzpp.py
```


## **Experiment Results**
AS-Fuzzer++ was evaluated in **Apollo 8.0** within the **Carla simulator**, demonstrating **substantial improvements** over AS-Fuzzer.  
Key findings include:

- **Identified 49 unique ADS failure cases**, including **37 previously undetected vulnerabilities**.
- **Test scenario generation efficiency improved by 25.8%**.
- **Interaction rates increased by 1.58×**, ensuring **higher realism in testing**.
- **Blocked scenario occurrences reduced by 32%**, leading to **more effective simulations**.

For details on **specific test cases**, see:  
[Key Scenarios Exposing Apollo Vulnerabilities](doc/key_scenarios.md).

---

## **Cite Our Work**
If you use AS-Fuzzer++ in your research, please cite our paper:

```bibtex
@inproceedings{
}

