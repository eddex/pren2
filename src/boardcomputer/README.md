# boardcomputer
[![Build Status](https://travis-ci.com/eddex/pren2.svg?token=iW3x5jyw6cyxqZrqYqGq&branch=master)](https://travis-ci.com/eddex/pren2)

The boardcomputer is a RaspberryPi 3 B+.

## project setup & usage
**initial setup**
1. create a virtual environment with python 3.6 in the `boardcomputer` folder, call it `venv` (PyCharm: `Fle > Settings > Project: boardcomputer > Project Interpreter`)
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
- the `tests` directories also need an `__init__.py` file

## testing
- use `pytest` from the `boardcomputer` root directory to run all tests
- file names for tests must start with `test_` otherwise the tests are not found by pytest
- test methods must start with `test_` otherwise the tests are not found by pytest

### mocks
- in unit tests all modules except for the one under test must be mocked. to achieve this, one might need to replace imported modules with mocks.
- how to mock imports: https://stackoverflow.com/questions/8658043/how-to-mock-an-import
- general introduction to mocks: http://www.drdobbs.com/testing/using-mocks-in-python/240168251

## coding style
- the [google python style guide](https://github.com/google/styleguide/blob/gh-pages/pyguide.md) is used for this project
- `yapf` can be used for auto formatting: https://github.com/google/yapf/#installation
- to setup pylint in PyCharm follow [this guide on stackoverflow](https://stackoverflow.com/questions/38134086/how-to-run-pylint-with-pycharm)

## software components

### camera
Takes pictures with the camera attached to the Raspberry Pi board.

### tms
The finite state machine for the boardcomputer.

### image_analysis
Analyze images to find different types of signals including start, stop and info signals.

### log
Log files are saved on the boradcomputer and (if connected) sent to an external device over network.

### uart_handler
Implementation of the UART protocol to communicate with the microcontroller.
