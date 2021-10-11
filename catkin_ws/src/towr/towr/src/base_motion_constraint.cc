/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/constraints/base_motion_constraint.h>
#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>
#include <towr/variables/spline_holder.h>
#include <iostream>

namespace towr {


BaseMotionConstraint::BaseMotionConstraint (double T, double dt,
                                            const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(T, dt, "baseMotion")
{
  base_linear_  = spline_holder.base_linear_;
  base_angular_ = spline_holder.base_angular_;
  double dev_rad = 0.05;
  double devz_rad = 0.15;

  node_bounds_.resize(k6D+1);

// x orientation - default
 // node_bounds_.at(AX) = Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AY) =  ifopt::NoBound;

// y orientation  -default
  //node_bounds_.at(AY) = Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AY) =  ifopt::NoBound;

// z orientation
   node_bounds_.at(AZ) = ifopt::NoBound;//Bounds(-dev_rad, dev_rad);

//------------------------------------------------------------------//

  double z_init = base_linear_->GetPoint(0.0).p().z();
  double y_init = base_linear_->GetPoint(0.0).p().y();

// x position bound
  node_bounds_.at(LX) = ifopt::NoBound;

// y position bound
 node_bounds_.at(LY) = ifopt::NoBound;//Bounds(-0.05, 0.05);  // original

// z position bound- default
  node_bounds_.at(LZ) = ifopt::NoBound;// allow to move dev_z cm up and down
   
  // 7th bound
  node_bounds_.at(LZ+1)=ifopt::BoundGreaterZero; // f_t1 > -mu*n


//------------------------------------------------------------------//
  int n_constraints_per_node = node_bounds_.size();
  SetRows(GetNumberOfNodes()*n_constraints_per_node);
}

void
BaseMotionConstraint::UpdateConstraintAtInstance (double t, int k,
                                                  VectorXd& g) const
{
// -- Original constraint 
   g.middleRows(GetRow(k, LX), k3D) = base_linear_->GetPoint(t).p();
   g.middleRows(GetRow(k, AX), k3D) = base_angular_->GetPoint(t).p();
  
//  //-- Adding a new constraint here for obstacles 

  double x_com = base_linear_->GetPoint(t).p().x();
  double y_com = base_linear_->GetPoint(t).p().y();
  double z_com = base_linear_->GetPoint(t).p().z();

  double delta = 0.05;
  double x0 = 1.5;
  double y0 = 0.0;
  double z0 = 0.0;

   g(GetRow(k, LZ+1)) = std::pow(std::pow(x0-x_com,2)+std::pow(y0-y_com,2),0.5)-delta;

}

void
BaseMotionConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{

  // Modified update bounds here-------------------------------------------------------
  //-----------------------------------------------------------------------------------

  // double dev_rad = 0.05;
  // double devz_rad = 0.15;
  // double y_init = base_linear_->GetPoint(0.0).p().y();

  // if (base_linear_->GetPoint(t).p().x() <= 1.5) {
  //   bounds.at(GetRow(k,AX)) = Bounds(-dev_rad, dev_rad);
  //   bounds.at(GetRow(k,AY)) = Bounds(-dev_rad, dev_rad);
  //   bounds.at(GetRow(k,AZ)) = Bounds(-devz_rad, devz_rad);

  //   bounds.at(GetRow(k,LX)) = ifopt::NoBound;
  //   bounds.at(GetRow(k,LY)) =  Bounds(y_init-0.25, y_init+0.25);//Bounds(-0.25, 0.25);
  //   bounds.at(GetRow(k,LZ)) = ifopt::NoBound;
  //   }
  
  // else if (((base_linear_->GetPoint(t).p().x() >=1.5)&&(base_linear_->GetPoint(t).p().x() <=3.0))&&(base_linear_->GetPoint(t).p().y()<=-0.5 )){
  
  //   //double x_curr = base_linear_->GetPoint(t).p().x();

  //   bounds.at(GetRow(k,AX)) = Bounds(-dev_rad, dev_rad);
  //   bounds.at(GetRow(k,AY)) = Bounds(-dev_rad, dev_rad);
  //   bounds.at(GetRow(k,AZ)) = ifopt::NoBound;

  //   bounds.at(GetRow(k,LX)) = Bounds(2.3-0.3, 2.3+0.3);//Bounds(-0.25, 0.25);
  //   bounds.at(GetRow(k,LY)) =  ifopt::NoBound;
  //   bounds.at(GetRow(k,LZ)) = ifopt::NoBound;


  // }
  
  
  // else {
  //   for (int dim=0; dim<node_bounds_.size(); ++dim) {
  //     bounds.at(GetRow(k,dim)) = node_bounds_.at(dim);
  //   }
  // }

  //-----------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------

  for (int dim=0; dim<node_bounds_.size(); ++dim) {
      bounds.at(GetRow(k,dim)) = node_bounds_.at(dim);
  }
  
  
}



void
BaseMotionConstraint::UpdateJacobianAtInstance (double t, int k,
                                                std::string var_set,
                                                Jacobian& jac) const
{

  double x_com = base_linear_->GetPoint(t).p().x();
  double y_com = base_linear_->GetPoint(t).p().y();
  double z_com = base_linear_->GetPoint(t).p().z();

  double x0 = 1.5;
  double y0 = 0.0;
  double z0 = 0.0;
  double temp1=0.0;

//  temp1 = std::pow(std::pow(x0-x_com,2)+std::pow(y0-y_com,2)+std::pow(z0-z_com,2),0.5);
  temp1 = -1.0*std::pow(std::pow(x0-x_com,2)+std::pow(y0-y_com,2),0.5);

  Eigen::Vector3d jac1;

  jac1(0) = (x0-x_com)/temp1;
  jac1(1) = (y0-y_com)/temp1;
  jac1(2) = 0.0;

  int n = 3;
  //Jacobian jac_model(1,n);

  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(3);
  for(int i=0; i<3;i++)
  {
    tripletList.push_back(T(0,i,jac1(i)));
  }
  Eigen::SparseMatrix<double> mat(1,n);
  mat.setFromTriplets(tripletList.begin(),tripletList.end());
  //jac_model = mat;

  if (var_set == id::base_ang_nodes){
    jac.middleRows(GetRow(k,AX), k3D) = base_angular_->GetJacobianWrtNodes(t, kPos);
  
    }

  if (var_set == id::base_lin_nodes)
    jac.middleRows(GetRow(k,LX), k3D) = base_linear_->GetJacobianWrtNodes(t, kPos);
    //jac.middleRows(GetRow(k,LZ+1),1) = mat;


}

int
BaseMotionConstraint::GetRow (int node, int dim) const
{
  return node*node_bounds_.size() + dim;
}

} /* namespace towr */
