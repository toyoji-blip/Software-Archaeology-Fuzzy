# Software-Archaeology-Fuzzy
Reconstruction and simulation of a 1987 fuzzy-tuned I-PD control system for software archaeology and LLM reliability benchmarking.
This repository contains two independent programs related to the reconstruction and evaluation of a fuzzy-tuned I-PD control system, originally implemented in 1987. This project serves as a technical artifact for Software Archaeology, facilitating the evaluation of deterministic logic compliance in modern Large Language Models (LLMs).

## 📁 Contents

### 1. Reconstruction Program (Full Design Pipeline)
This program reconstructs the original control design workflow from the 1987 study:
- Process Identification: Automated identification using the moment method.
- Model Approximation: Reduction of complex processes to a second-order plus dead-time model.
- Tuning: PID parameter generation via fuzzy inference (Legacy Logic).
- Structure: Implementation of the I-PD control structure.
- Simulation: Verification of the complete design pipeline through control response simulation.

### 2. Comparison Program (Performance Evaluation)
This program evaluates disturbance rejection performance using predefined PID parameters. It compares three distinct tuning methods:
- Fuzzy-based tuning (The 1987 Legacy Logic)
- TCM (Takahashi–Chan Method, 1971)
- IAE Minimization (Integral Absolute Error)

The program simulates and compares responses across multiple process models:
- 3rd-order and 4th-order lag processes.
- 2nd-order and 3rd-order processes with zeros.

*Note: This program focuses on performance evaluation and does not perform active tuning.*

---

## 🖥️ How to Run

The programs are implemented in standalone JavaScript (ES2026), designed for ease of execution without complex build environments.

### Option 1: Google Gemini Canvas (Author's Setup)
1. Open Google Gemini.
2. Paste the source code into the Canvas editor.
3. Switch to Preview mode to execute the simulation and view the graphs.

### Option 2: Web Browser (Recommended for Users)
Since the programs are browser-compatible:
1. Save the source code as an `.html` file (wrapping the JS in `<script>` tags).
2. Open the file in any modern web browser (Chrome, Edge, or Firefox).
3. The simulation results and interactive charts will render automatically.

---

## 📚 References

If you use this code or refer to the underlying research, please cite the following:

- [1] Kanagawa, T. (2026).Deterministic Compliance Failures in Large Language Models: A Structural Analysis Using a Legacy Fuzzy Inference Benchmark. engrXiv. https://doi.org/10.31224/6702
- [2] Kanagawa, T. (1987).** A Study on PID Auto-Tuning Control Using Fuzzy Inference. Master’s Thesis, Toyohashi University of Technology.

---

## ⚖️ License
This project is licensed under the MIT License. See the `LICENSE` file for details.
