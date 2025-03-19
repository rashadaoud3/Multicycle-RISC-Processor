# 🏗️ Multicycle RISC Processor

This repository contains the **design, implementation, and verification** of a **16-bit pipelined RISC processor** using **Verilog**. The processor is designed as part of the **Computer Architecture** course and follows a multicycle execution model.

## 📌 Project Objectives:
✔ **Design the Datapath & Control Path** – Implement a 16-bit RISC processor with a multicycle architecture.  
✔ **Instruction Execution** – Support R-type, I-type, and J-type instructions.  
✔ **Simulation & Testing** – Verify the functionality using testbenches and assembly programs.  

## 🛠 Processor Specifications:
- **16-bit instruction and word size**  
- **8 general-purpose 16-bit registers**  
- **Dedicated 16-bit PC (Program Counter) & RR (Return Register)**  
- **Supports R-type, I-type, and J-type instructions**  
- **Word-addressable memory**  
- **Performance tracking registers** for execution analysis  

## 📂 Instruction Set:
| Instruction | Type | Operation |
|------------|------|-----------|
| `AND`  | R-Type | Bitwise AND |
| `ADD`  | R-Type | Addition |
| `SUB`  | R-Type | Subtraction |
| `SLL`  | R-Type | Shift left |
| `SRL`  | R-Type | Shift right |
| `ANDI` | I-Type | Bitwise AND with immediate |
| `ADDI` | I-Type | Addition with immediate |
| `LW`   | I-Type | Load word |
| `SW`   | I-Type | Store word |
| `BEQ`  | I-Type | Branch if equal |
| `BNE`  | I-Type | Branch if not equal |
| `FOR`  | I-Type | Loop execution |
| `JMP`  | J-Type | Unconditional jump |
| `CALL` | J-Type | Function call |
| `RET`  | J-Type | Return from function |

## 🔧 Tools & Technologies Used:
- **Verilog** – Hardware description language for processor implementation.  
- **ModelSim / Quartus** – Simulation and verification.  

## 📄 Project Components:
- **Datapath Design** – Registers, ALU, Memory, Control Unit.  
- **Control Path Implementation** – Instruction decoding, execution logic.  
- **Multicycle Stages** – Fetch, Decode, Execute, Memory, Write-back.  
- **Simulation & Testing** – Custom testbench and sample programs.  

## 🚀 How to Run:
1. Load the Verilog files into **ModelSim or Quartus**.  
2. Compile the design and run the provided **testbench**.  
3. Verify instruction execution using **waveform analysis**.  

## 📜 Report:
A **detailed project report (PDF)** is included, covering:  
✔ Design and implementation details  
✔ Block diagrams & state diagrams  
✔ Control signals & truth tables  
✔ Simulation results and test programs  

This project demonstrates **pipelined processor design** and its verification in **Verilog**. 🏗️🚀  
