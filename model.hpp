#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <string>
#include <math.h>

//Important: all calculations done assuming SI units 
class VacuumModel
{
   private:
      double init_height;
      double total_mass;
      double g; 

      double height;
      double velocity;
      double acceleration;
      double net_force;
      double time;
      double time_incrementer;
      int n_steps;

   public:
      VacuumModel();

      void setInitialHeight(double init_height);
      void setMass(double total_mass);
      void setGravitationalAcceleration(double g);
      void setTimeIncrementer(double time_incrementer);

      double getInitialHeight();
      double getMass();
      double getGravitationalAcceleration();
      double getTimeIncrementer();

      //Functions corrosponding to variables that will change over time
      double getTimeElapsed();
      double getCurrentHeight();
      double getCurrentVelocity();
      double getCurrentAcceleration();
      double getCurrentNetForce();

      //Functions for computation and predictive purposes
      virtual double predictAcceleration(double time);
      virtual double predictNetForce(double time);
      virtual double predictVelocity(double time);
      virtual double predictHeight(double time);

      //Increment system with time or reset the system back to initial conditions
      void evolveSystem();
      void resetSystem();
      void printStatus();

};

class LinearAirResistanceModel : public VacuumModel
{
   private:
      double b_constant;
      std::string shape;

   public:
      LinearAirResistanceModel();
      void setBConstant(double b_constant);
      void setShape(std::string shape);

      double getBConstant();
      std::string getShape();
      
      double predictVelocity(double time);
      double predictNetForce(double time);
      double predictAcceleration(double time);
      double predictHeight(double time);
      double calculateTerminalVelocity();

};

class QuadraticAirResistanceModel : public VacuumModel
{
   private:
      //Initialization constants
      double air_density; // kilogram per metre cubed
      double cross_sectional_area; // metres squared
      std::string shape; //shape of parachute (i.e. square, circle, etc)
      double coeff_of_drag; 

   public:
      //constructor
      QuadraticAirResistanceModel();

      //Getter and setter functions corrosponding to constants after parachute is initialized
      void setAirDensity(double air_density);
      void setCrossSectionalArea(double cross_section_area);
      void setShape(std::string shape);
      void setCoefficientOfDrag(double coeff_of_drag);

      double getAirDensity();
      double getCrossSectionalArea();
      std::string getShape();
      double getCoefficientOfDrag();

      double predictVelocity(double time);
      double predictNetForce(double time);
      double predictAcceleration(double time);
      double predictHeight(double time);
      double calculateTerminalVelocity();

};

#endif
