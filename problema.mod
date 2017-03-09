
set J; #insieme dei prodotti
set D ordered; #insieme delle dimensioni
set R ordered; # insieme delle rotazioni

param s{J,D,R} ; #la dimensione dell'oggetto J, nella dimensione D nella rotazione R
param po{J} ; #profitto ricavato dalla scelta dell'oggetto J
param S{D} ; #dimensione dello zaino nella dimensione D
param BigM;
param pu{J} ;
param pv{j in J}=s[j,first(D),first(R)]*s[j,member(2,D),first(R)]*s[j,last(D),first(R)]; #volume di ogni oggetto
param min_po= min {i in J} pv[i];#oggetto piu'piccolo
param new_po{j in J}=pv[j]/min_po; 


var t{i in J} binary; #binaria che assume valore 1 se  si prende l'oggetto j
var b{i in J, j in J, d in D} binary;#binaria che assume valore 1 se l'oggetto i precede l'oggetto j nella dimensione d 
var p{i in J, k in R} binary; #binaria che assume valore 1 se l'oggetto j e' usato nella rotazione R
var X{j in J, d in D} integer;


#maximize profitto_compact:sum{j in J}(po[j]*t[j]);

#maximize profitto_compact:(S[first(D)]*S[member(2,D)]*S[last(D)])*(sum{j in J} (pu[j]*t[j])) - sum{i in J} sum{d in D}X[i,d];

maximize profitto_compact:(S[first(D)]*S[member(2,D)]*S[last(D)])*(sum{j in J} (pv[j]*t[j])) - sum{i in J} sum{d in D}X[i,d];

#maximize profitto_compact:(S[first(D)]*S[member(2,D)]*S[last(D)]) *( sum{j in J} (new_po[j]*t[j])) - sum{i in J}sum{d in D}X[i,d] ;

subject to capienza: sum {j in J} s[j,first(D),first(R)]*s[j,member(2,D),first(R)]*s[j,last(D),first(R)]*t[j]<=
S[first(D)]*S[member(2,D)]*S[last(D)];


subject to knapsack_limit{d in D ,i in J}: X[i,d]+ sum {r in R} s[i,d,r]*p[i,r] <= S[d];

subject to coordinate_control {d in D ,i in J}: X[i,d]<= BigM*t[i];

subject to limit_position_i{d in D ,i in J, j in J: i<j}: X[i,d]+ sum {r in R} s[i,d,r]*p[i,r] <= X[j,d] + BigM *(1-b[i,j,d]);

subject to limit_position_j{d in D ,i in J, j in J: i<j}: X[j,d]+ sum {r in R} s[j,d,r]*p[j,r] <= X[i,d] + BigM *(1-b[j,i,d]); 

subject to controllo_adiacente_i{d in D ,i in J, j in J}: b[i,j,d]<=t[i];

subject to controllo_adiacente_j{d in D ,i in J, j in J}: b[j,i,d]<=t[j];

subject to overlap {j in J ,i in J}: sum {d in D} (b[i,j,d] + b[j,i,d]) >= t[i]+t[j]-1;

subject to limit_rotation{i in J}: sum {r in R} p[i,r] <= 1*t[i];

subject to min_rotation{i in J}: sum{r in R} p[i,r] >=t[i];

subject to limit_spigolo{i in J, d in D}:X[i,d] >= 0;

