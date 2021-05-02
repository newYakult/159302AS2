#include <algorithm>
#include "fuzzylogic.h"

//Yamakawa's exsample implementation
const int NUM_OR_RULES = 13;
const int NUM_OF_INPUT = 2;
const int NUM_OF_INPUT_REGION = 5;
const int NUM_OF_OUTPUT = 7;


/////////////////////////////////////////////////////////////////
struct In_agl_params {
   static constexpr float nm[4] = {-1.5, -0.5, 0, 0};
   static constexpr float ns[4] = {-1.2, -1, -0.7, -0};
   static constexpr float ze[4] ={-0.7, 0, 0, 0.7}; 
   static constexpr float ps[4] = {0, 0.7, 2, 1.2};
   static constexpr float pm[4] = {0.5, 1.5, 0, 0};
};


struct In_pos_params {
   static constexpr float nm[4] = { -1.5, -0.5, 0, 0 };
   static constexpr float ns[4] = { -1.2, -1, -0.8, -0 };
   static constexpr float ze[4] = { -0.7, 0, 0, 0.7 };
   static constexpr float ps[4] = { 0, 0.7, 2, 1.2 };
   static constexpr float pm[4] = { 0.5, 1.5, 0, 0 };
};

//Initialise Fuzzy Rules

void initFuzzyRules(fuzzy_system_rec *fl) {
//---------------------------------------------------------------------------- 	
//THETA vs. THETA_DOT	
//   
   for (int i = 0;i < NUM_OR_RULES;i++) {
       fl->rules[i].inp_index[0] = in_agl;
       fl->rules[i].inp_index[1] = in_pos;
   }
   // Rules matrix (13 rules):
   //-----------------------------
   // in_pos/in_agl:
   //              nm   ns    ze    ps    pm
   //             __________________________
   //          pm |ns         ps          pl
   //          ps |     ns          pm    
   //          ze |nm         ze          pm
   //          ns |     nm          ps       
   //          nm |nl         ns          ps
   //--------------------------------
   fl->rules[0].inp_fuzzy_set[in_agl] = in_nm;
   fl->rules[0].inp_fuzzy_set[in_pos] = in_pm;
   fl->rules[0].out_fuzzy_set = out_ns;
	
   fl->rules[1].inp_fuzzy_set[in_agl] = in_ze;
   fl->rules[1].inp_fuzzy_set[in_pos] = in_pm;
   fl->rules[1].out_fuzzy_set = out_ps;

   fl->rules[2].inp_fuzzy_set[in_agl] = in_pm;
   fl->rules[2].inp_fuzzy_set[in_pos] = in_pm;
   fl->rules[2].out_fuzzy_set = out_pl;

   fl->rules[3].inp_fuzzy_set[in_agl] = in_ns;
   fl->rules[3].inp_fuzzy_set[in_pos] = in_ps;
   fl->rules[3].out_fuzzy_set = out_ns;
	
   fl->rules[4].inp_fuzzy_set[in_agl] = in_ps;
   fl->rules[4].inp_fuzzy_set[in_pos] = in_ps;
   fl->rules[4].out_fuzzy_set = out_pm;
	
   fl->rules[5].inp_fuzzy_set[in_agl] = in_nm;
   fl->rules[5].inp_fuzzy_set[in_pos] = in_ze;
   fl->rules[5].out_fuzzy_set = out_nm;
	
   fl->rules[6].inp_fuzzy_set[in_agl] = in_ze;
   fl->rules[6].inp_fuzzy_set[in_pos] = in_ze;
   fl->rules[6].out_fuzzy_set = out_ze;
	
   fl->rules[7].inp_fuzzy_set[in_agl] = in_pm;
   fl->rules[7].inp_fuzzy_set[in_pos] = in_ze;
   fl->rules[7].out_fuzzy_set = out_pm;
	
   fl->rules[8].inp_fuzzy_set[in_agl] = in_ns;
   fl->rules[8].inp_fuzzy_set[in_pos] = in_ns;
   fl->rules[8].out_fuzzy_set = out_nm;
	
   fl->rules[9].inp_fuzzy_set[in_agl] = in_ps;
   fl->rules[9].inp_fuzzy_set[in_pos] = in_ns;
   fl->rules[9].out_fuzzy_set = out_ps;
	
   fl->rules[10].inp_fuzzy_set[in_agl] = in_nm;
   fl->rules[10].inp_fuzzy_set[in_pos] = in_nm;
   fl->rules[10].out_fuzzy_set = out_nl;
	
   fl->rules[11].inp_fuzzy_set[in_agl] = in_ze;
   fl->rules[11].inp_fuzzy_set[in_pos] = in_nm;
   fl->rules[11].out_fuzzy_set = out_ns;
	
   fl->rules[12].inp_fuzzy_set[in_agl] = in_pm;
   fl->rules[12].inp_fuzzy_set[in_pos] = in_nm;
   fl->rules[12].out_fuzzy_set = out_ps;

   return;
}


void initMembershipFunctions(fuzzy_system_rec *fl) {
	
   fl -> inp_mem_fns[in_agl][in_nm] = init_trapz(In_agl_params::nm, left_trapezoid);
   fl -> inp_mem_fns[in_agl][in_ns] = init_trapz(In_agl_params::ns, regular_trapezoid);
   fl -> inp_mem_fns[in_agl][in_ze] = init_trapz(In_agl_params::ze, regular_trapezoid);
   fl -> inp_mem_fns[in_agl][in_ps] = init_trapz(In_agl_params::ps, regular_trapezoid);
   fl -> inp_mem_fns[in_agl][in_pm] = init_trapz(In_agl_params::pm, right_trapezoid);

   fl -> inp_mem_fns[in_pos][in_nm] = init_trapz(In_agl_params::nm, left_trapezoid);
   fl -> inp_mem_fns[in_pos][in_ns] = init_trapz(In_agl_params::ns, regular_trapezoid);
   fl -> inp_mem_fns[in_pos][in_ze] = init_trapz(In_agl_params::ze, regular_trapezoid);
   fl -> inp_mem_fns[in_pos][in_ps] = init_trapz(In_agl_params::ps, regular_trapezoid);
   fl -> inp_mem_fns[in_pos][in_pm] = init_trapz(In_agl_params::pm, right_trapezoid);
	
   return;
}

void initFuzzySystem (fuzzy_system_rec *fl, float A, float B, float C, float D, unsigned small_f, unsigned medium_f, unsigned large_f) {
   fl->A = A;
   fl->B = B;
   fl->C = C;
   fl->D = D;
   //Note: The settings of these parameters will depend upon your fuzzy system design
   fl->no_of_inputs = NUM_OF_INPUT; 
   fl->no_of_inp_regions = NUM_OF_INPUT_REGION;
   fl->no_of_rules = NUM_OR_RULES; //define a rule for every region combination between inputs

   fl->no_of_outputs = NUM_OF_OUTPUT; 
	fl->output_values [out_nl] = -1.0 * large_f;
	fl->output_values [out_nm] = -1.0 * medium_f;
	fl->output_values [out_ns] = -1.0 * large_f;
	fl->output_values [out_ze] = 0.0;
	fl->output_values [out_ps] = 1.0 * small_f;
	fl->output_values [out_pm] = 1.0 * medium_f;
	fl->output_values [out_pl] = 1.0 * large_f;

   

   fl->rules = (rule *) malloc ((size_t)(fl->no_of_rules*sizeof(rule)));
   initFuzzyRules(fl);
   initMembershipFunctions(fl);
   return;
}

//////////////////////////////////////////////////////////////////////////////

trapezoid init_trapz (const float key_points[4], trapz_type typ) {
	
   trapezoid trz;
   trz.a = key_points[0];
   trz.b = key_points[1];
   trz.c = key_points[2];
   trz.d = key_points[3];
   trz.tp = typ;
   switch (trz.tp) {
	   
      case regular_trapezoid:
         	 trz.l_slope = 1.0/(trz.b - trz.a);
         	 trz.r_slope = 1.0/(trz.c - trz.d);
         	 break;
	 
      case left_trapezoid:
         	 trz.r_slope = 1.0/(trz.a - trz.b);
         	 trz.l_slope = 0.0;
         	 break;
	 
      case right_trapezoid:
         	 trz.l_slope = 1.0/(trz.b - trz.a);
         	 trz.r_slope = 0.0;
         	 break;
   }  /* end switch  */
   
   return trz;
}  /* end function */

//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
float trapz (float x, trapezoid trz) {
   switch (trz.tp) {
	   
      case left_trapezoid:
         	 if (x <= trz.a)
         	    return 1.0;
         	 if (x >= trz.b)
         	    return 0.0;
         	 /* a < x < b */
         	 return trz.r_slope * (x - trz.b);
	 
	 
      case right_trapezoid:
         	 if (x <= trz.a)
         	    return 0.0;
         	 if (x >= trz.b)
         	    return 1.0;
         	 /* a < x < b */
         	 return trz.l_slope * (x - trz.a);
	 
      case regular_trapezoid:
         	 if ((x <= trz.a) || (x >= trz.d))
         	    return 0.0;
         	 if ((x >= trz.b) && (x <= trz.c))
         	    return 1.0;
         	 if ((x >= trz.a) && (x <= trz.b))
         	    return trz.l_slope * (x - trz.a);
         	 if ((x >= trz.c) && (x <= trz.d))
         	    return  trz.r_slope * (x - trz.d);
         	    
	 }  /* End switch  */
	 
   return 0.0;  /* should not get to this point */
}  /* End function */

//////////////////////////////////////////////////////////////////////////////
float min_of(float values[],int no_of_inps) {
   int i;
   float val;
   val = values [0];
   for (i = 1;i < no_of_inps;i++) {
       if (values[i] < val)
	  val = values [i];
   }
   return val;
}



//////////////////////////////////////////////////////////////////////////////
float fuzzy_system (float inputs[],fuzzy_system_rec fz) {
   int i,j;
   short variable_index,fuzzy_set;
   float sum1 = 0.0,sum2 = 0.0,weight;
   float m_values[MAX_NO_OF_INPUTS];
	
   float fuzzy_inputs[2] = {(fz.A*inputs[0] + fz.B*inputs[1]), (fz.C*inputs[2] + fz.D*inputs[3])};
   for (i = 0;i < fz.no_of_rules;i++) {
      for (j = 0;j < fz.no_of_inputs;j++) {
	   variable_index = fz.rules[i].inp_index[j];
      //cout << "Variable index: " << variable_index << endl;
	   fuzzy_set = fz.rules[i].inp_fuzzy_set[j];
      //cout << "Fuzzy set:" << fuzzy_set << endl;
      trapezoid trp = fz.inp_mem_fns[variable_index][fuzzy_set];
      //cout << "Trapzoid number " << fuzzy_set << " " << trp.a << "," << trp.b << "," << trp.c << "," << trp.d << endl;
	   m_values[j] = trapz(fuzzy_inputs[variable_index],
	       trp);
	   } /* end j  */
      
       weight = min_of (m_values,fz.no_of_inputs);
				
       sum1 += weight * fz.output_values[fz.rules[i].out_fuzzy_set];
       sum2 += weight;
   } /* end i  */
 
	
	if (fabs(sum2) < TOO_SMALL) {
      //cout << "\r\nFLPRCS Error: Sum2 in fuzzy_system is 0.  Press key: " << endl;
      //~ getch();
      //~ exit(1);
      return 0.0;
   }
   
   return (sum1/sum2);
}  /* end fuzzy_system  */

//////////////////////////////////////////////////////////////////////////////
void free_fuzzy_rules (fuzzy_system_rec *fz) {
   if (fz->allocated){
	   free (fz->rules);
	}
	
   fz->allocated = false;
   return;
}

