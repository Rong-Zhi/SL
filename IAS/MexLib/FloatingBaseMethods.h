
#ifndef FLOATINGBASEMETHODS_H_
#define FLOATINGBASEMETHODS_H_

int slStartIndex = 1;

void init(double* controller_gain_th,double* controller_gain_thd,double* controller_gain_int)
{
        if (!read_gains(config_files[GAINS],controller_gain_th,controller_gain_thd, controller_gain_int))
        	return;
        
        // inverse dynamics
	if (!init_dynamics())
		return;  
        
        // initialize kinematics
        init_kinematics();
        
        // object handling
        if(!initObjects())
        	return;
        
        // assign contact force mappings
        #include "LEKin_contact.h"
        
        // need sensor offsets
        if (!read_sensor_offsets(config_files[SENSOROFFSETS]))
        	return;
    
}


void initJointSimState(double* q_des, double* q_sim, double* controller_gain_th, double* controller_gain_thd)
{
    	for(int i=0; i < N_DOFS; i++) {
			joint_sim_state[i+slStartIndex].th = q_sim[i];
			joint_sim_state[i+slStartIndex].thd = q_sim[N_DOFS + i];
            //joint_sim_state[i+slStartIndex].thdd = q_sim[2*N_DOFS + i];
            joint_sim_state[i+slStartIndex].thdd = 0;
            

            joint_sim_state[i+slStartIndex].u = (q_des[i] - q_sim[i])*controller_gain_th[i+slStartIndex] + (q_des[N_DOFS + i] - q_sim[N_DOFS + i])*controller_gain_thd[i+slStartIndex];
            
            // check torque constraints
            if(fabs(joint_sim_state[i+slStartIndex].u) > u_max[i+slStartIndex]){
                joint_sim_state[i+slStartIndex].u = macro_sign(joint_sim_state[i+slStartIndex].u) * u_max[i+slStartIndex];
            }        
		}
}

void initBase(double* base, double* orient)
{
    for(int i  = 0; i < N_CART ; i++){
        	base_state.x[i+slStartIndex] = base[i];
            base_state.xd[i+slStartIndex] = base[i + N_CART];
            base_state.xdd[i+slStartIndex] = base[i + 2*N_CART];
            
            base_orient.ad[i+slStartIndex] = orient[i + 3*N_QUAT];
            base_orient.add[i+slStartIndex] = orient[i + 3*N_QUAT + N_CART];
    }  
        
    for(int i  = 0; i < N_QUAT ; i++){
             base_orient.q[i+slStartIndex] = orient[i];
            base_orient.qd[i+slStartIndex] = orient[i + N_QUAT];
            base_orient.qdd[i+slStartIndex] = orient[i + 2*N_QUAT];
    }
}

void checkTorqueLimits(double* controller_gain_th, double* controller_gain_thd)
{
    double k,kd;
    double delta;
    double max_vel = 10.;
    double aux;
    
    for (int i=1; i<=N_DOFS; ++i) {

            // use Geyer limited rebound model for (set aux=1 to avoid)
            k  = controller_gain_th[i]*10.0;
            kd = controller_gain_thd[i]*sqrt(10.0);

            delta = joint_sim_state[i].th - joint_range[i][MIN_THETA];

            if ( delta < 0 ){
      
                if (joint_sim_state[i].thd > max_vel) 
                    aux = 0.0;
                else 
                    aux = 1.-joint_sim_state[i].thd/max_vel;

                joint_sim_state[i].u += - delta * k * aux - joint_sim_state[i].thd * kd * aux;
            }

            delta = joint_range[i][MAX_THETA] - joint_sim_state[i].th;
            if (delta < 0){ 

                if (joint_sim_state[i].thd < -max_vel) 
                    aux = 0.0;
                else 
                    aux = 1.-joint_sim_state[i].thd/(-max_vel);
 
                joint_sim_state[i].u += delta * k * aux - joint_sim_state[i].thd * kd * aux;
            }
        }
}

void computeBase(double dt, int n_integration)
{
	double dt_step = dt / n_integration;
        
        for(int i = 1; i<=n_integration; ++i){  
            SL_IntegrateEuler(joint_sim_state,&base_state,&base_orient,ucontact,endeff,dt_step,N_DOFS,TRUE);
        }

}

void cleanup()
{
	
	// clean up all defined variables in init_kinematics
	int i;

	my_free_matrix(link_pos,0,N_LINKS,1,3);
	my_free_matrix(link_pos_des,0,N_LINKS,1,3);
	my_free_matrix(link_pos_sim,0,N_LINKS,1,3);
	my_free_matrix(joint_cog_mpos,0,N_DOFS,1,3);
	my_free_matrix(joint_cog_mpos_des,0,N_DOFS,1,3);
	my_free_matrix(joint_cog_mpos_sim,0,N_DOFS,1,3);
	my_free_matrix(joint_origin_pos,0,N_DOFS,1,3);
	my_free_matrix(joint_origin_pos_des,0,N_DOFS,1,3);
	my_free_matrix(joint_origin_pos_sim,0,N_DOFS,1,3);
	my_free_matrix(joint_axis_pos,0,N_DOFS,1,3);
	my_free_matrix(joint_axis_pos_des,0,N_DOFS,1,3);
	my_free_matrix(joint_axis_pos_sim,0,N_DOFS,1,3);

	my_free_matrix(J,1,N_ENDEFFS*6,1,N_DOFS);
	my_free_matrix(dJdt,1,N_ENDEFFS*6,1,N_DOFS);
	my_free_matrix(Jdes,1,N_ENDEFFS*6,1,N_DOFS);
	my_free_matrix(Jcog,1,2*N_CART,1,N_DOFS);
	my_free_matrix(Jcogdes,1,2*N_CART,1,N_DOFS);

	my_free_matrix(Jbase,1,N_ENDEFFS*6,1,6);
	my_free_matrix(dJbasedt,1,N_ENDEFFS*6,1,6);
	my_free_matrix(Jbasedes,1,N_ENDEFFS*6,1,6);
	my_free_matrix(Jbasecog,1,2*N_CART,1,2*N_CART);
	my_free_matrix(Jbasecogdes,1,2*N_CART,1,2*N_CART);
	
	for (i=0; i<=N_LINKS; ++i) {
		my_free_matrix(Alink[i],1,4,1,4);
		my_free_matrix(Alink_des[i],1,4,1,4);
		my_free_matrix(Alink_sim[i],1,4,1,4);	
	}

	for (i=0; i<=N_DOFS; ++i) {
		my_free_matrix(Adof[i],1,4,1,4);
		my_free_matrix(Adof_des[i],1,4,1,4);
		my_free_matrix(Adof_sim[i],1,4,1,4);	
	}

	// clean up defined variables in initObjects
	free(contacts);
	free(ucontact);

}


#endif /* FLOATINGBASEMETHODS_H_ */
