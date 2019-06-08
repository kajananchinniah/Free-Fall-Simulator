#include "model.hpp"

//Note: the constructors are mostly used to provide initial values to everything; everything should be overwritten

//VacuumModel class constructor, and functions 
VacuumModel::VacuumModel()
{
   this->init_height = 5.0;
   this->total_mass = 1.0;
   this->g = 9.81;

   this->height = 5.0;
   this->velocity = 0.0;
   this->acceleration = 0.0;
   this->net_force = 0.0; 
   this->time = 0.0;
   this->time_incrementer = 0.1;
   this->n_steps = 0;
}

void VacuumModel::setInitialHeight(double init_height)
{
   if (init_height < 0.0)
   {
      std::cout << "Error: cannot have negative height... Taking absolute value.\n";
      init_height = init_height * -1.0;
   }

   this->init_height = init_height;
   this->height = init_height;
}

void VacuumModel::setMass(double total_mass)
{
   if (total_mass == 0.0)
   {
      std::cout << "Error: cannot have 0 kilograms of mass! Exiting....\n";
      exit(-1);
   }

   if (total_mass < 0.0)
   {
      std::cout << "Error: cannot have negative mass... Taking absolute value.\n";
      total_mass = total_mass * -1.0;
   }

   this->total_mass = total_mass;
}

void VacuumModel::setGravitationalAcceleration(double g)
{
   if (g == 0.0)
   {
      std::cout << "Error: cannot have an acceleration due to gravity of 0! Exiting...";
      exit(-1);
   }

   if (g < 0.0)
   {
      g = -1.0 * g;
   }

   this->g = g;
}

void VacuumModel::setTimeIncrementer(double time_incrementer)
{
   if (time_incrementer == 0.0)
   {
      std::cout << "Error! Time MUST increment. Using value of 0.1 for incrementer...\n";
      time_incrementer = 0.1;
   }

   if (time_incrementer < 0.0)
   {
      std::cout << "Error: cannot go backwards in time... Taking negative version of the input\n";
      time_incrementer = -1.0 * time_incrementer; 
   }

   this->time_incrementer = time_incrementer;
}

double VacuumModel::getInitialHeight()
{
   return this->init_height;
}

double VacuumModel::getMass()
{
   return this->total_mass;
}

double VacuumModel::getGravitationalAcceleration()
{
   return this->g;
}

double VacuumModel::getTimeIncrementer()
{
   return this->time_incrementer;
}

double VacuumModel::getTimeElapsed()
{
   return this->time;
}

double VacuumModel::getCurrentHeight()
{
   return this->height;
}

double VacuumModel::getCurrentVelocity()
{
   return this->velocity;
}

double VacuumModel::getCurrentAcceleration()
{
   return this->acceleration;
}

double VacuumModel::getCurrentNetForce()
{
   return this->net_force;
}

double VacuumModel::predictAcceleration(double time)
{
   double pred_acceleration = this->g;
   return pred_acceleration;
}

double VacuumModel::predictNetForce(double time)
{
   double m = this->total_mass;
   double a = this->acceleration;

   double pred_net_force = m * a;
   return pred_net_force;
}

double VacuumModel::predictVelocity(double time)
{
   double a = this->acceleration;

   double pred_velocity = a * time;
   return pred_velocity;
}

double VacuumModel::predictHeight(double time)
{
   double a = this->acceleration;
   double h = this->height;

   double pred_height = -0.5 * a * time * time + h;
   return pred_height;
}

void VacuumModel::evolveSystem()
{
   this->time = this->time + this->time_incrementer;
   this->velocity = this->predictVelocity(this->time);
   this->net_force = this->predictNetForce(this->time);
   this->acceleration = this->predictAcceleration(this->time);
   this->height = this->predictHeight(this->time);

   if (this->getCurrentHeight() <= 0.0)
   {
      std::cout << "Object reached ground!\n";
      //Object is no longer moving at this point
      this->height = 0.0;
      this->velocity = 0.0;
      this->acceleration = 0.0;
      this->net_force = 0.0;
   }

}

void VacuumModel::resetSystem()
{
   this->time = 0.0;
   this->height = this->init_height;
   this->velocity = 0.0;
   this->acceleration = this->g;
   this->net_force = this->g * this->total_mass;
}

void VacuumModel::printStatus()
{
   std::cout << "---------------------------------\n";
   std::cout << "Time = " << this->time << " s\n";
   std::cout << "Height = " << this->height << " m\n";
   std::cout << "Velocity = " << this->velocity << " m/s downwards\n";
   std::cout << "Acceleration = " << this->acceleration << " m/s^2 downwards\n";
   std::cout << "Net Force = " << this->net_force << " N downwards\n";
   std::cout << "---------------------------------\n";
}


LinearAirResistanceModel::LinearAirResistanceModel()
{
   this->b_constant = 1.0;
   this->shape = "circle";
}

void LinearAirResistanceModel::setBConstant(double b_constant)
{
   if (b_constant == 0.0)
   {
      std::cout << "Error: The constant b cannot be zero! Exiting...\n";
      exit(-1);
   }

   if (b_constant < 0.0)
   {
      std::cout << "Error: the constant b is less than 0! Taking absolute of the value...\n";
      b_constant = -1.0 * b_constant;
   }
   this->b_constant = b_constant;
}

void LinearAirResistanceModel::setShape(std::string shape)
{
   this->shape = shape;
}

double LinearAirResistanceModel::getBConstant()
{
   return this->b_constant;
}

std::string LinearAirResistanceModel::getShape()
{
   return this->shape;
}

double LinearAirResistanceModel::predictVelocity(double time)
{
   double m = this->getMass();
   double g = this->getGravitationalAcceleration();
   double b = this->b_constant;
   double v_t = (m*g)/b;
   double tau = m/b;

   double v = v_t * (1 - exp(-1.0 * time / tau)); 

   if (isnan(v))
   {
      v = this->calculateTerminalVelocity();
   }

   return v;
}

double LinearAirResistanceModel::predictNetForce(double time)
{
   double m = this->getMass();
   double g = this->getGravitationalAcceleration();
   double b = this->b_constant;
   double curr_v = this->predictVelocity(time);

   double pred_net_force = m * g - b * curr_v; 
   
   if (isnan(pred_net_force))
   {
      return 0.0;
   }

   return pred_net_force;
}

double LinearAirResistanceModel::predictAcceleration(double time)
{
   double F = this->predictNetForce(time);
   double m = this->getMass();

   double pred_acceleration = F / m;
   
   if (isnan(pred_acceleration))
   {
      return 0.0;
   }

   return pred_acceleration;
}

double LinearAirResistanceModel::predictHeight(double time)
{
   double t = time;
   double h_0 = this->getInitialHeight();

   double m = this->getMass();
   double b = this->getBConstant();
   double g = this->getGravitationalAcceleration();
   double v_t = this->calculateTerminalVelocity();

   double exp_param = (-1.0 * g * t) / v_t;

   double term_1 = h_0;
   double term_2 = v_t * time;
   double term_3 = (v_t * v_t / g) * (1.0 - exp(exp_param));
   return term_1 - term_2 + term_3;
}

double LinearAirResistanceModel::calculateTerminalVelocity()
{
   double m = this->getMass();
   double g = this->getGravitationalAcceleration();
   double b = this->getBConstant();

   double terminal_velocity = (m * g) / b;
   return terminal_velocity;
}

QuadraticAirResistanceModel::QuadraticAirResistanceModel()
{
   this->air_density = 1.225; //density of air at 20 degrees C
   this->cross_sectional_area = 1.0;
   this->shape = "circle"; 
   this->coeff_of_drag = 1.0;
}

void QuadraticAirResistanceModel::setAirDensity(double air_density)
{
   if (air_density == 0.0)
   {
      std::cout << "Error: cannot have air density of 0.... Exiting.\n";
      exit(-1);
   }
   
   if (air_density < 0.0)
   {
      std::cout << "Error: negative air density was inputted... Taking absolute value\n";
      air_density = -1.0 * air_density;
   }

   this->air_density = air_density;
}

void QuadraticAirResistanceModel::setCrossSectionalArea(double cross_sectional_area)
{
   if (cross_sectional_area == 0)
   {
      std::cout << "Error: cross sectional area cannot be zero...\n";
      exit(-1);
   }

   if (cross_sectional_area < 0)
   {
      std::cout << "Error: negative cross sectional area was inputted... Taking absolute value\n";
      cross_sectional_area = -1.0 * cross_sectional_area;
   }

   this->cross_sectional_area = cross_sectional_area;
}

void QuadraticAirResistanceModel::setShape(std::string shape)
{
   this->shape = shape;
}

void QuadraticAirResistanceModel::setCoefficientOfDrag(double coeff_of_drag)
{
   if (coeff_of_drag == 0.0)
   {
      std::cout << "Error: cannot have coefficient of drag equal to 0! Exiting...\n";
      exit(-1);
   }

   if (coeff_of_drag < 0)
   {
      std::cout << "Taking absolute value of coefficient of drag...\n";
      coeff_of_drag = -1.0 * coeff_of_drag;
   }
   this->coeff_of_drag = coeff_of_drag;
}

double QuadraticAirResistanceModel::getAirDensity()
{
  return this->air_density; 
}

double QuadraticAirResistanceModel::getCrossSectionalArea()
{
   return this->cross_sectional_area;
}

std::string QuadraticAirResistanceModel::getShape()
{
   return this->shape;
}

double QuadraticAirResistanceModel::getCoefficientOfDrag()
{
   return this->coeff_of_drag;
}

double QuadraticAirResistanceModel::predictVelocity(double time)
{
   double c = (this->coeff_of_drag * this->air_density * this->cross_sectional_area) / 2.0;
   double g = this->getGravitationalAcceleration();
   double m = this->getMass();
   double exp_param = 2 * time * sqrt( (g*c)/m );
   double v_num = sqrt(g) * exp(exp_param) - sqrt(g);
   double v_dem = sqrt(c/m) * (1 + exp(exp_param));
   double v_t = v_num / v_dem;
   
   if (isnan(v_t))
   {
      v_t = this->calculateTerminalVelocity();
   }

   return v_t;
}

double QuadraticAirResistanceModel::predictNetForce(double time)
{
   double c = (this->coeff_of_drag * this->air_density * this->cross_sectional_area) / 2.0;
   double vel_pred = this->predictVelocity(time);
   double F_air = c * vel_pred * vel_pred;
   double F_g = this->getGravitationalAcceleration()  * this->getMass();
   
   if (isnan(F_air) || isnan(F_g))
   {
      return 0.0;
   }

   return F_g - F_air;
}

double QuadraticAirResistanceModel::predictAcceleration(double time)
{
   double F_net = this->predictNetForce(time);
   
   if (isnan(F_net))
   {
      return 0.0;
   }

   return F_net / this->getMass();
}

//Current this is solved using rienmann sums
double QuadraticAirResistanceModel::predictHeight(double time)
{
   double t = time;
   double h_0 = this->getInitialHeight();
   double v_t = this->calculateTerminalVelocity();
   double g = this->getGravitationalAcceleration();
   double term_1 = h_0;
   double term_2 = (v_t * v_t / g) * log(std::cosh(g*t/v_t));
   return term_1 - term_2;

}

double QuadraticAirResistanceModel::calculateTerminalVelocity()
{
   double m = this->getMass();
   double g = this->getGravitationalAcceleration();

   double terminal_velocity_sq = (2.0 * g * m) / (this->coeff_of_drag * this->air_density * this->cross_sectional_area);
   double terminal_velocity = sqrt(terminal_velocity_sq);
   return terminal_velocity;
}


