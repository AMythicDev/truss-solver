# Truss solver using Python

Solves a truss problem using the joints method. The following information are required for running this
- Trusses: Which joint is connected to which joint
- Joints: Properties about each joint such as its X, Y coordinate, reaction forces along X and Y axis etc

Please read the INPUTS section in main.py to see a detailed explanation on how to set this data.

It will respond with the load in each truss and reaction forces where any pin joint/roller joint exists.

**USAGE**:

It is expected that the host running this has Python and Pip installed.
- Clone the repositiory
```sh
git clone https://github.com/AMythicDev/truss-solver.git
```

- Install the requirements
```sh
pip install -r requirements.txt
```

- Enter the data in main.py
- Run main.py
```
python main.py
```

**NOTE BEFORE USAGE**:
- This tool is only a proof of concept and is not intended for professional use. Please use one of the
online tools if you have such use cases.
- It does not provide any sort of interface for end-users to enter data. You are expected to enter the data
directly in the main.py source file.
- The tool requires atleast one joint to have 2 unknown trusses. It will not work if all joints have more than 2
unknowns.

**NOTE ABOUT CODE**:

This is a college assignment and I only gave a doy or two to write this. Hence it contains a hell of
unoptimized algorithms and hacky code everywhere to \*just\* get the job done. It is also largely
undocumented.

The user of it is expected to enter correct data as there are lot of code paths that are unchecked and can produce
unexpected results if the data isn't entered properly.

The code does not have any error handling whatsoever so any wrong usage would lead to a Python exception.

Do not file issues/PR regarding the aforementioned reasons. Send issues/PRs only if it imtroduces more physical parameters
such as cross-sectional area, Young's modulus etc or improves the physics related algorithms
or helps in finding other physical parameters such as shear or stress.

**LICENSE**:

The code available under the MIT License and all contributions shall be under the same license unless explicitly mentioned.
