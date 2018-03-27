/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
 
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gsAHPbar _p[0]
#define ik _p[1]
#define gk _p[2]
#define c _p[3]
#define ek _p[4]
#define casi _p[5]
#define cinf _p[6]
#define ctau _p[7]
#define Dc _p[8]
#define v _p[9]
#define _g _p[10]
#define _ion_ek	*_ppvar[0]._pval
#define _ion_ik	*_ppvar[1]._pval
#define _ion_dikdv	*_ppvar[2]._pval
#define _ion_casi	*_ppvar[3]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_NE2(void);
 static void _hoc_NE1(void);
 static void _hoc_cbet(void);
 static void _hoc_calf(void);
 static void _hoc_rate(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_sAHPNE", _hoc_setdata,
 "NE2_sAHPNE", _hoc_NE2,
 "NE1_sAHPNE", _hoc_NE1,
 "cbet_sAHPNE", _hoc_cbet,
 "calf_sAHPNE", _hoc_calf,
 "rate_sAHPNE", _hoc_rate,
 0, 0
};
#define NE2 NE2_sAHPNE
#define NE1 NE1_sAHPNE
#define cbet cbet_sAHPNE
#define calf calf_sAHPNE
 extern double NE2( _threadargsprotocomma_ double );
 extern double NE1( _threadargsprotocomma_ double );
 extern double cbet( _threadargsprotocomma_ double , double );
 extern double calf( _threadargsprotocomma_ double , double );
 /* declare global and static user variables */
#define NE_t2 NE_t2_sAHPNE
 double NE_t2 = 0.7;
#define NE_start2 NE_start2_sAHPNE
 double NE_start2 = 36000;
#define NE_period2 NE_period2_sAHPNE
 double NE_period2 = 100;
#define NE_ext2 NE_ext2_sAHPNE
 double NE_ext2 = 212000;
#define NE_ext1 NE_ext1_sAHPNE
 double NE_ext1 = 196000;
#define NE_t1 NE_t1_sAHPNE
 double NE_t1 = 0.9;
#define NE_stop NE_stop_sAHPNE
 double NE_stop = 96000;
#define NE_start NE_start_sAHPNE
 double NE_start = 64000;
#define NE_period NE_period_sAHPNE
 double NE_period = 500;
#define tone_period tone_period_sAHPNE
 double tone_period = 4000;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "ik_sAHPNE", "mA/cm2",
 "gk_sAHPNE", "mho/cm2",
 0,0
};
 static double c0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "tone_period_sAHPNE", &tone_period_sAHPNE,
 "NE_period_sAHPNE", &NE_period_sAHPNE,
 "NE_start_sAHPNE", &NE_start_sAHPNE,
 "NE_stop_sAHPNE", &NE_stop_sAHPNE,
 "NE_t1_sAHPNE", &NE_t1_sAHPNE,
 "NE_ext1_sAHPNE", &NE_ext1_sAHPNE,
 "NE_ext2_sAHPNE", &NE_ext2_sAHPNE,
 "NE_period2_sAHPNE", &NE_period2_sAHPNE,
 "NE_start2_sAHPNE", &NE_start2_sAHPNE,
 "NE_t2_sAHPNE", &NE_t2_sAHPNE,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[4]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"sAHPNE",
 "gsAHPbar_sAHPNE",
 0,
 "ik_sAHPNE",
 "gk_sAHPNE",
 0,
 "c_sAHPNE",
 0,
 0};
 static Symbol* _k_sym;
 static Symbol* _cas_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 11, _prop);
 	/*initialize range parameters*/
 	gsAHPbar = 2.31814e-005;
 	_prop->param = _p;
 	_prop->param_size = 11;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0]._pval = &prop_ion->param[0]; /* ek */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 prop_ion = need_memb(_cas_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[3]._pval = &prop_ion->param[1]; /* casi */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _sahpNE_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	ion_reg("cas", 2.0);
 	_k_sym = hoc_lookup("k_ion");
 	_cas_sym = hoc_lookup("cas_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 11, 5);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 sAHPNE I:/SkyDrive/Nair Lab/Osci/LFP/single_cell/sahpNE.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rate(_threadargsprotocomma_ double, double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[1], _dlist1[1];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   rate ( _threadargscomma_ v , casi ) ;
   Dc = ( cinf - c ) / ctau ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 rate ( _threadargscomma_ v , casi ) ;
 Dc = Dc  / (1. - dt*( ( ( ( - 1.0 ) ) ) / ctau )) ;
 return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   rate ( _threadargscomma_ v , casi ) ;
    c = c + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / ctau)))*(- ( ( ( cinf ) ) / ctau ) / ( ( ( ( - 1.0) ) ) / ctau ) - c) ;
   }
  return 0;
}
 
double calf ( _threadargsprotocomma_ double _lv , double _lcasi ) {
   double _lcalf;
 double _lvs , _lva ;
  _lvs = 10.0 * log10 ( 1000.0 * _lcasi ) ;
   _lcalf = 0.0048 / exp ( - 0.5 * ( _lvs - 35.0 ) ) ;
    
return _lcalf;
 }
 
static void _hoc_calf(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  calf ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
double cbet ( _threadargsprotocomma_ double _lv , double _lcasi ) {
   double _lcbet;
 double _lvs , _lvb ;
  _lvs = 10.0 * log10 ( 1000.0 * _lcasi ) ;
   _lcbet = 0.012 / exp ( 0.2 * ( _lvs + 100.0 ) ) ;
    
return _lcbet;
 }
 
static void _hoc_cbet(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  cbet ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
static int  rate ( _threadargsprotocomma_ double _lv , double _lcasi ) {
   double _lcsum , _lca , _lcb ;
  _lca = calf ( _threadargscomma_ _lv , _lcasi ) ;
   _lcb = cbet ( _threadargscomma_ _lv , _lcasi ) ;
   _lcsum = _lca + _lcb ;
   if ( _lv < - 65.0 ) {
     cinf = 0.0 ;
     }
   else {
     cinf = _lca / _lcsum ;
     }
   ctau = 48.0 ;
     return 0; }
 
static void _hoc_rate(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 rate ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
double NE1 ( _threadargsprotocomma_ double _lt ) {
   double _lNE1;
 if ( _lt >= NE_start  && _lt <= NE_stop ) {
     if ( ( _lt / tone_period - floor ( _lt / tone_period ) ) >= ( 1.0 - NE_period / tone_period ) ) {
       _lNE1 = NE_t1 ;
       }
     else if ( ( _lt / tone_period - floor ( _lt / tone_period ) )  == 0.0 ) {
       _lNE1 = NE_t1 ;
       }
     else {
       _lNE1 = 1.0 ;
       }
     }
   else if ( _lt >= NE_ext1  && _lt <= NE_ext2 ) {
     if ( ( _lt / tone_period - floor ( _lt / tone_period ) ) >= ( 1.0 - NE_period / tone_period ) ) {
       _lNE1 = NE_t1 ;
       }
     else if ( ( _lt / tone_period - floor ( _lt / tone_period ) )  == 0.0 ) {
       _lNE1 = NE_t1 ;
       }
     else {
       _lNE1 = 1.0 ;
       }
     }
   else {
     _lNE1 = 1.0 ;
     }
   
return _lNE1;
 }
 
static void _hoc_NE1(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  NE1 ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double NE2 ( _threadargsprotocomma_ double _lt ) {
   double _lNE2;
 if ( _lt >= NE_start2  && _lt <= NE_stop ) {
     if ( ( _lt / tone_period - floor ( _lt / tone_period ) ) >= ( 1.0 - NE_period2 / tone_period ) ) {
       _lNE2 = NE_t2 ;
       }
     else if ( ( _lt / tone_period - floor ( _lt / tone_period ) )  == 0.0 ) {
       _lNE2 = NE_t2 ;
       }
     else {
       _lNE2 = 1.0 ;
       }
     }
   else {
     _lNE2 = 1.0 ;
     }
   
return _lNE2;
 }
 
static void _hoc_NE2(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  NE2 ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ek = _ion_ek;
  casi = _ion_casi;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 1; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ek = _ion_ek;
  casi = _ion_casi;
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_k_sym, _ppvar, 0, 0);
   nrn_update_ion_pointer(_k_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 2, 4);
   nrn_update_ion_pointer(_cas_sym, _ppvar, 3, 1);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  c = c0;
 {
   rate ( _threadargscomma_ v , casi ) ;
   c = cinf ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  ek = _ion_ek;
  casi = _ion_casi;
 initmodel(_p, _ppvar, _thread, _nt);
 }}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   gk = gsAHPbar * c ;
   ik = gk * ( v - ek ) * NE1 ( _threadargscomma_ t ) * NE2 ( _threadargscomma_ t ) ;
   }
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  ek = _ion_ek;
  casi = _ion_casi;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
 double _break, _save;
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _break = t + .5*dt; _save = t;
 v=_v;
{
  ek = _ion_ek;
  casi = _ion_casi;
 { {
 for (; t < _break; t += dt) {
   states(_p, _ppvar, _thread, _nt);
  
}}
 t = _save;
 } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(c) - _p;  _dlist1[0] = &(Dc) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
