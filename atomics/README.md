# Atomic Models

## QSS Integrator

This atomic model features a QSS numerical integrator.

* Inputs: 
  * `IN_dx`: time derivative of `x`.
* Outputs: 
  * `OUT_q`: quantized state.
* Parameters:
  * `dQMin`: Minimum quantum. Default value: 1e-6.
  * `dQRel`: Relative quantum. Default value: 1e-3.
  * `x0`: Initial condition. Default value: 0.
  * `gain`: Input gain. Default value: 1.
  * `debug`: Ff True the atomic model prints out debug information. Default value: False.

## Collector

This atomic model logs in a CSV file the events it receives in its input ports. Add as many input ports as needed and edit the external transition function to log every incoming event.

  * Inputs:
    * `in1_event`: Events input 1.
    * `in2_event`: Events input 2.
  * Parameters:
    * `filename`: Name of the logging file. Default value: `output.csv`.
