# boardcomputer
The boardcomputer is a RaspberryPi 3 B+.

## project setup & usage
**initial setup**
1. create a virtual environment with python 3.6 in the `boardcomputer` folder, call it `venv`
2. change to the environment in your (powershell) console: `.\venv\Scripts\activate`
3. update pip: `easy_install -U pip`
4. install requirements: `pip install -r requirements.txt`

if there are errors regarding your execution policy, run powershell as admin and execute `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`

**adding a package**
1. change to the virtual environment in your console
2. `pip install <package-name>`
3. `pip freeze > requirements.txt`

## project structure
- each component has its own directory.
- add `__init__.py` for each component
- each component must have a `tests` subdirectory for unit tests
- the `tests` directories also need an `__init__.py file

## testing
- use `pytest` from the `boardcomputer` root directory to run all tests
- test methods must start with `test_` other wise the tests are not found by pytest

## software components

### logging
Log files are saved on the boradcomputer and (if connected) sent to an external device over network.

### image recognition
Different signals have to be recognized alongside the rails.

### communication with the microcontroller
UART