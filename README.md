# Real T
## _Overview_:
This project is a mixed HW and SW system allowing to geolocalize in real time objects or persons.using STM32F4 and SIM808 EVB 3.2. 
## _Concept_:
The project has two main parts:
- Software:
design of the prototype and the application part which, together, form our prototype. The system operates in two modes: 
-- The first mode is about tracing the route in real time and displaying it on the android app "GPS Demo" accessible by the location requester
-- The second mode performs by receiving the location by the SMS service which allows to benefit from the services of the system without resorting to an internet connection given the need to take into account the discontinuous coverages of the internet or GPRS services in certain zones while taking care to reduce the electric power consumption of the system.
- Hardware: implementation of the different circuits in order to study the energy aspects of the different components and justify the reasons for the choice of the best circuit and we have visualized the tracings on the Android application.
