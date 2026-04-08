/*
 This program reproduces a control system design originally implemented in C language in 1987.

 It includes:

 1. Process identification using the moment method
 2. PID parameter tuning via fuzzy inference

 The implementation is kept structurally consistent with the original logic.
*/


import { useState, useEffect, useCallback, useRef } from "react";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, ReferenceLine } from "recharts";

// ============================================================
// IDENTIFICATION MODULE
// ============================================================
const getIdDt = () => 0.01;

const step2nd = (K, T1, T2, L, dt, dur) => {
  const N=Math.floor(dur/dt), Ld=Math.max(1,Math.round(L/dt)), d=T1-T2;
  let p1,p2,b1,b2;
  if(Math.abs(d)<1e-6){p1=Math.exp(-dt/T1);p2=p1;b1=K*(1-p1*(1+dt/T1));b2=K*p1*(p1-1+(dt/T1)*p1);}
  else{p1=Math.exp(-dt/T1);p2=Math.exp(-dt/T2);b1=K*(1-(T1*p1-T2*p2)/d);b2=K*(p1*p2-(T1*p2-T2*p1)/d);}
  let z1=0,z2=0; const buf=new Array(Ld+2).fill(0),data=[];
  for(let k=0;k<=N;k++){
    const y=(p1+p2)*z1-p1*p2*z2+b1*buf[Ld]+b2*buf[Ld+1];
    data.push({t:+(k*dt).toFixed(5),y:Math.max(0,y)});
    for(let j=Ld+1;j>0;j--)buf[j]=buf[j-1];
    buf[0]=1;z2=z1;z1=y;
  }
  return data;
};

const step2ndZero=(K,T1,T2,Tz,L,dt,dur)=>{
  const N=Math.floor(dur/dt),Ld=Math.max(0,Math.round(L/dt));
  const p1=Math.exp(-dt/T1),p2=Math.exp(-dt/T2),A=K*(1-Tz/T1)/(1-T2/T1),B=K*(1-Tz/T2)/(1-T1/T2);
  let y1=0,y2=0;const buf=new Array(Math.max(Ld+2,2)).fill(0),data=[];
  for(let k=0;k<=N;k++){
    const u=buf[Ld],ny1=p1*y1+A*(1-p1)*u,ny2=p2*y2+B*(1-p2)*u;
    data.push({t:+(k*dt).toFixed(5),y:Math.max(0,ny1+ny2)});
    for(let j=Ld;j>0;j--)buf[j]=buf[j-1];
    buf[0]=1;y1=ny1;y2=ny2;
  }
  return data;
};

const step3rd=(K,Ta,Tb,Tc,L,dt,dur)=>{
  const N=Math.floor(dur/dt),Ld=Math.max(0,Math.round(L/dt));
  const pa=Math.exp(-dt/Ta),pb=Math.exp(-dt/Tb),pc=Math.exp(-dt/Tc);
  let ya=0,yb=0,yc=0;const buf=new Array(Math.max(Ld+2,2)).fill(0),data=[];
  for(let k=0;k<=N;k++){
    const u=buf[Ld],nya=pa*ya+K*(1-pa)*u,nyb=pb*yb+(1-pb)*nya,nyc=pc*yc+(1-pc)*nyb;
    data.push({t:+(k*dt).toFixed(5),y:Math.max(0,nyc)});
    for(let j=Ld;j>0;j--)buf[j]=buf[j-1];
    buf[0]=1;ya=nya;yb=nyb;yc=nyc;
  }
  return data;
};

const step3rdZero=(K,Ta,Tb,Tc,Tz,L,dt,dur)=>{
  const N=Math.floor(dur/dt),Ld=Math.max(0,Math.round(L/dt));
  const sa=-1/Ta,sb=-1/Tb,sc=-1/Tc;
  const A=K*(Tz*sa+1)/((Tb*sa+1)*(Tc*sa+1)),B=K*(Tz*sb+1)/((Ta*sb+1)*(Tc*sb+1)),C=K*(Tz*sc+1)/((Ta*sc+1)*(Tb*sc+1));
  const pa=Math.exp(-dt/Ta),pb=Math.exp(-dt/Tb),pc=Math.exp(-dt/Tc);
  let ya=0,yb=0,yc=0;const buf=new Array(Math.max(Ld+2,2)).fill(0),data=[];
  for(let k=0;k<=N;k++){
    const u=buf[Ld],nya=pa*ya+A*(1-pa)*u,nyb=pb*yb+B*(1-pb)*u,nyc=pc*yc+C*(1-pc)*u;
    data.push({t:+(k*dt).toFixed(5),y:Math.max(0,nya+nyb+nyc)});
    for(let j=Ld;j>0;j--)buf[j]=buf[j-1];
    buf[0]=1;ya=nya;yb=nyb;yc=nyc;
  }
  return data;
};

const step4th=(K,Ta,Tb,Tc,Td,L,dt,dur)=>{
  const N=Math.floor(dur/dt),Ld=Math.max(0,Math.round(L/dt));
  const pa=Math.exp(-dt/Ta),pb=Math.exp(-dt/Tb),pc=Math.exp(-dt/Tc),pd=Math.exp(-dt/Td);
  let ya=0,yb=0,yc=0,yd=0;const buf=new Array(Math.max(Ld+2,2)).fill(0),data=[];
  for(let k=0;k<=N;k++){
    const u=buf[Ld],nya=pa*ya+K*(1-pa)*u,nyb=pb*yb+(1-pb)*nya,nyc=pc*yc+(1-pc)*nyb,nyd=pd*yd+(1-pd)*nyc;
    data.push({t:+(k*dt).toFixed(5),y:Math.max(0,nyd)});
    for(let j=Ld;j>0;j--)buf[j]=buf[j-1];
    buf[0]=1;ya=nya;yb=nyb;yc=nyc;yd=nyd;
  }
  return data;
};

const computeMoments=(data,dt,Cinf)=>{
  let sTm=0;
  for(let i=0;i<data.length-1;i++)sTm+=((Cinf-data[i].y)+(Cinf-data[i+1].y))/2*dt;
  const Tm=sTm/Cinf;
  let sTs=0,sTa=0;
  for(let i=0;i<data.length-1;i++){
    const tm=(data[i].t+data[i+1].t)/2,ym=(data[i].y+data[i+1].y)/2;
    sTs+=(tm-Tm)*(Cinf-ym)*dt;sTa+=(tm-Tm)**2*(Cinf-ym)*dt;
  }
  return{Tm,Ts2:Tm*Tm+2*sTs/Cinf,Ta3:3*sTa/Cinf-Tm**3};
};

const solveAlpha=(beta)=>{
  const f=a=>2*(1+a**3)-beta*(1+a*a)**1.5;
  let lo=0.0001,hi=0.9999,found=false;
  for(let i=0;i<10000;i++){const a0=0.0001+i*0.9998/10000;if(f(a0)*f(a0+0.9998/10000)<=0){lo=a0;hi=a0+0.9998/10000;found=true;break;}}
  if(!found){let bA=0.5,bV=Math.abs(f(0.5));for(let i=1;i<=10000;i++){const a=0.0001+i*0.9998/10000,v=Math.abs(f(a));if(v<bV){bV=v;bA=a;}}return{alpha:bA,residual:bV};}
  for(let i=0;i<150;i++){const m=(lo+hi)/2;if(Math.abs(hi-lo)<1e-14)break;if(f(lo)*f(m)<=0)hi=m;else lo=m;}
  return{alpha:(lo+hi)/2,residual:Math.abs(f((lo+hi)/2))};
};

const runId=(type,p,tau)=>{
  const dt=getIdDt();
  const tS=type==="2nd"||type==="2ndZero"?p.T1+p.T2+p.L:type==="3rd"||type==="3rdZero"?p.Ta+p.Tb+p.Tc+p.L:p.Ta+p.Tb+p.Tc+p.Td+p.L;
  const dur=50*tS;
  const data=type==="2nd"?step2nd(p.K,p.T1,p.T2,p.L,dt,dur):type==="2ndZero"?step2ndZero(p.K,p.T1,p.T2,p.Tz,p.L,dt,dur):type==="3rd"?step3rd(p.K,p.Ta,p.Tb,p.Tc,p.L,dt,dur):type==="3rdZero"?step3rdZero(p.K,p.Ta,p.Tb,p.Tc,p.Tz,p.L,dt,dur):step4th(p.K,p.Ta,p.Tb,p.Tc,p.Td,p.L,dt,dur);
  const ts=Math.floor(data.length*0.9);
  const Cinf=data.slice(ts).reduce((s,d)=>s+d.y,0)/(data.length-ts);
  const{Tm,Ts2,Ta3}=computeMoments(data,dt,Cinf);
  if(Ts2<=0)return null;
  const Ts=Math.sqrt(Ts2),Ta=Math.sign(Ta3)*Math.abs(Ta3)**(1/3),beta=(Ta/Ts)**3;
  const{alpha,residual}=solveAlpha(beta);
  const r1=Math.sqrt(Ts2/(1+alpha*alpha)),r2=alpha*r1;
  // const T1id=Math.min(r1,r2),T2id=Math.max(r1,r2),Lid=Math.max(0,Tm-T1id-T2id);
  const T1id=r1, T2id=r2, Lid=Math.max(0,Tm-T1id-T2id);
  return{K:Cinf,T1:T1id,T2:T2id,L:Lid,tau,Tm,alpha,beta,residual};
};

// ============================================================
// FUZZY TABLES
// ============================================================
const ALPHA_TABLES={
  alpha1:[[0,2],[2,2.5],[2.5,3],[3,3.5],[3.5,4],[4,4.5],[4.5,5],[5,5.5],[5.5,6],[6,6.5],[6.5,7],[7,7.5],[7.5,8],[8,8.5],[8.5,9],[9,9.5],[9.5,10],[10,12],[12,15],[15,18],[18,20],[20,25],[25,30],[30,35],[35,50],[50,999999]],
  alpha2:[[0,2],[2,2.5],[2.5,3],[3,3.5],[3.5,4],[4,4.5],[4.5,5],[5,5.5],[5.5,6],[6,6.5],[6.5,7],[7,7.5],[7.5,8],[8,8.5],[8.5,9],[9,9.5],[9.5,10],[10,12],[12,15],[15,18],[18,20],[20,25],[25,30],[30,35],[35,50],[50,999999]],
  alpha3:[[0,100],[100,200],[200,400],[400,600],[600,1000],[1000,10000],[10000,1000000]],
  alpha4:[[0,100],[100,300],[300,500],[500,1000],[1000,5000],[5000,10000],[10000,50000],[50000,1000000]],
  alpha5:[[0,0.1],[0.1,0.5],[0.5,1.0],[1.0,5.0],[5.0,10.0],[10.0,1000]]
};
const out_kp_universe=[[0,2,0],[2,2.5,2],[2.5,3,2.5],[3,3.5,3],[3.5,4,3.5],[4,4.5,4],[4.5,5,4.5],[5,5.5,5],[5.5,6,5.5],[6,6.5,6],[6.5,7,6.5],[7,7.5,7],[7.5,8,7.5],[8,8.5,8],[8.5,9,8.5],[9,9.5,9],[9.5,10,9.5],[10,12,10],[12,15,12],[15,18,15],[18,20,18],[20,25,20],[25,30,25],[30,35,30],[35,999,35]];
const out_ki_universe=[[0,2,0],[2,2.5,2],[2.5,3,2.5],[3,3.5,3],[3.5,4,3.5],[4,4.5,4],[4.5,5,4.5],[5,5.5,5],[5.5,6,5.5],[6,6.5,6],[6.5,7,6.5],[7,7.5,7],[7.5,8,7.5],[8,8.5,8],[8.5,9,8.5],[9,9.5,9],[9.5,10,9.5],[10,12,10],[12,15,12],[15,18,15],[18,20,18],[20,25,20],[25,30,25],[30,35,30],[35,999,35]];
const OUT_KD_TYPE1=[0,7,20,30,37,45,55,70];
const OUT_KD_TYPE2=[0,4,10,15,20,25,30,50,80];
const membership_alpha4=[[0,0,0,0,0,0,0.2,1],[0,0,0,0,0.2,0.8,1,0.5],[0,0,0.2,0.8,1,0.8,0.2,0],[0,0.2,1,0.8,0.2,0,0,0]];
const membership_alpha3=[[0,0,0,0,0.2,0.8,1],[0,0.1,0.5,0.7,1,0.8,0.2],[0,0.2,0.8,1,0.8,0.2,0],[0.2,0.7,1,0.8,0.2,0,0]];
const membership_alpha2=[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.9,1.0,1.0,1.0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1.0,0.9,0.7,0.2,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1.0,0.9,0.7,0.5,0.2,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,0.7,0.9,1.0,0.9,0.7,0.2,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1.0,0.9,0.5,0.2,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1.0,0.9,0.7,0.5,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1.0,0.9,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0],[0,0,0,0.2,0.5,0.7,0.8,0.9,1.0,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0.2,0.5,0.7,0.9,1.0,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0.5,0.7,0.8,1.0,0.9,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.7,1.0,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]];
const membership_alpha1=[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.9,0.9,1,1],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1,0.9,0.7,0.2,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1,0.9,0.7,0.5,0.2,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,0.7,0.9,1,0.9,0.7,0.2,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1,0.9,0.5,0.2,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1,0.9,0.9,0.8,0.7,0.2,0,0,0,0,0,0,0,0],[0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0],[0,0,0.2,0.5,0.7,0.8,0.9,0.9,1,0.9,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0],[0,0.2,0.5,0.8,0.9,0.9,1,0.9,0.9,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.2,0.7,0.9,1,0.9,0.8,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.7,1,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]];
const membership_delta_kp=[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1.0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,1.0,0.9,0.7],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.9,1.0,0.9,0.7,0.2,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.9,1.0,0.9,0.7,0.2,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1.0,0.9,0.5,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.8,0.9,1.0,0.9,0.9,0.7,0.2,0,0,0,0,0,0],[0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,0.9,1.0,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0],[0,0,0,0,0.2,0.5,0.7,0.8,0.8,1.0,0.9,0.9,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0],[0,0,0.2,0.5,0.7,0.9,0.9,1.0,0.9,0.8,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0],[0,0.5,0.7,0.8,0.9,1.0,0.8,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.5,0.8,1.0,0.9,0.9,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]];
const membership_delta_ki=[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.9,0.9],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1,0.9,0.7,0.2],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0.7,0.9,1,0.9,0.7,0.5,0.2,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,0.7,0.9,1,0.9,0.7,0.2,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1,0.9,0.5,0.2,0,0,0,0,0,0],[0,0,0,0,0,0,0,0.2,0.5,0.7,0.8,0.9,1,0.9,0.9,0.8,0.7,0.2,0,0,0,0,0,0,0,0],[0,0,0,0,0,0.2,0.5,0.7,0.9,0.9,1,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0],[0,0,0,0.2,0.5,0.7,0.8,0.9,1,0.9,0.9,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0],[0,0.2,0.5,0.7,0.9,0.9,1,0.8,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.5,0.7,0.9,1,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.7,1,0.9,0.8,0.7,0.5,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]];
const membership_kd1_table57=[[0,0,0,0,0,0.1,0.5,1],[0,0,0,0.2,0.5,0.8,1,0.5],[0,0,0.2,0.7,1,0.7,0.2,0],[0,0.5,1,0.8,0.2,0,0,0]];
const membership_kd2_table511=[[0,0,0,0,0.2,0.8,1,0.7,0.2],[0,0,0.2,0.7,1,0.8,0.2,0,0],[0,0.2,0.8,1,0.8,0.2,0,0,0],[0.2,0.7,1,0.8,0.2,0,0,0,0]];

const getIdx=(val,table)=>{for(let i=0;i<table.length;i++){const[mn,mx]=table[i];if(val>=mn&&val<mx)return i;}return table.length-1;};
const gT1=(lr,tbl)=>{let mx=-1,si=0,mc=0;for(let i=0;i<8;i++){const mu=tbl[lr][i];if(mu>mx+0.0001){mx=mu;si=i;mc=1;}else if(mu>=mx-0.0001&&mx>0){si+=i;mc++;}}return mc===0?0:Math.round(si/mc);};
const rT1=(a4,a5,a5u,a4u,mA4,mKd1,oKd1)=>{const i5=getIdx(a5,a5u);if(i5<3)return 0;const i4=getIdx(a4,a4u);let mx=-1;const lb=[];for(let r=0;r<4;r++){const mu=mA4[r][i4];if(mu>mx)mx=mu;}if(mx<0.8)return 0;for(let r=0;r<4;r++){const mu=mA4[r][i4];if(mu>=mx-0.0001){lb.push(r);if(lb.length>=2)break;}}const i1=gT1(lb[0],mKd1);if(lb.length===2){const i2=gT1(lb[1],mKd1);return oKd1[Math.floor((i1+i2)/2+0.9)];}return oKd1[i1];};
const gT2=(lr,tbl)=>{let mx=-1,si=0,mc=0;for(let i=0;i<9;i++){const mu=tbl[lr][i];if(mu>mx+0.0001){mx=mu;si=i;mc=1;}else if(mu>=mx-0.0001&&mx>0){si+=i;mc++;}}return mc===0?0:Math.floor(si/mc+0.5);};
const rT2=(a3,a3u,mA3,mKd2,oKd2)=>{const i3=getIdx(a3,a3u);let mx=-1;const lb=[];for(let r=0;r<4;r++){const mu=mA3[r][i3];if(mu>mx)mx=mu;}if(mx<0.8)return 0;for(let r=0;r<4;r++){const mu=mA3[r][i3];if(mu>=mx-0.0001){lb.push(r);if(lb.length>=2)break;}}const i1=gT2(lb[0],mKd2);if(lb.length===2){const i2=gT2(lb[1],mKd2);return oKd2[Math.floor((i1+i2)/2+0.9)];}return oKd2[i1];};
const gT3=(lr,tbl)=>{let mx=-1,si=0,mc=0;for(let i=0;i<tbl[lr].length;i++){const mu=tbl[lr][i];if(mu>mx+0.0001){mx=mu;si=i;mc=1;}else if(mu>=mx-0.0001&&mx>0){si+=i;mc++;}}return mc===0?0:Math.floor(si/mc+0.5);};
const rT3=(a2,a2u,mA2,mDkp,oKp)=>{const i2=getIdx(a2,a2u);let mx=-1;const lb=[];for(let r=0;r<mA2.length;r++){const mu=mA2[r][i2];if(mu>mx)mx=mu;}if(mx<0.8)return 0;for(let r=0;r<mA2.length;r++){const mu=mA2[r][i2];if(mu>=mx-0.0001){lb.push(r);if(lb.length>=2)break;}}const i1=gT3(lb[0],mDkp);if(lb.length===2){const i2b=gT3(lb[1],mDkp);return oKp[Math.floor((i1+i2b)/2+0.9)][2];}return oKp[i1][2];};
const gT4=(lr,tbl)=>{let mx=-1,si=0,mc=0;for(let i=0;i<tbl[lr].length;i++){const mu=tbl[lr][i];if(mu>mx+0.0001){mx=mu;si=i;mc=1;}else if(mu>=mx-0.0001&&mx>0){si+=i;mc++;}}return mc===0?0:Math.floor(si/mc+0.5);};
const rT4=(a1,a1u,mA1,mDki,oKi)=>{const i1=getIdx(a1,a1u);let mx=-1;const lb=[];for(let r=0;r<mA1.length;r++){const mu=mA1[r][i1];if(mu>mx)mx=mu;}if(mx<0.8)return 0;for(let r=0;r<mA1.length;r++){const mu=mA1[r][i1];if(mu>=mx-0.0001){lb.push(r);if(lb.length>=2)break;}}const i1b=gT4(lb[0],mDki);if(lb.length===2){const i2=gT4(lb[1],mDki);return oKi[Math.floor((i1b+i2)/2+0.9)][2];}return oKi[i1b][2];};

// ============================================================
// UI CONSTANTS
// ============================================================
const TYPE_LIST=[["2nd","2次遅れ"],["2ndZero","2次+零点"],["3rd","3次遅れ"],["3rdZero","3次+零点"],["4th","4次遅れ"]];
const FIELDS={
  "2nd":[["K","K",0.1],["T1","T₁",0.1],["T2","T₂",0.1],["L","L",0.1]],
  "2ndZero":[["K","K",0.1],["T1","T₁(極)",0.1],["T2","T₂(極)",0.1],["Tz","Tz(零点)",0.1,true],["L","L",0.1]],
  "3rd":[["K","K",0.1],["Ta","Tₐ",0.1],["Tb","T_b",0.1],["Tc","T_c",0.1],["L","L",0.1]],
  "3rdZero":[["K","K",0.1],["Ta","Tₐ(極)",0.1],["Tb","T_b(極)",0.1],["Tc","T_c(極)",0.1],["Tz","Tz(零点)",0.1,true],["L","L",0.1]],
  "4th":[["K","K",0.1],["Ta","Tₐ",0.1],["Tb","T_b",0.1],["Tc","T_c",0.1],["Td","T_d",0.1],["L","L",0.1]],
};
const DEF_PARAMS={
  "2nd":{K:1,T1:0.6,T2:1,L:1},
  "2ndZero":{K:1,T1:2,T2:5,Tz:1,L:2},
  "3rd":{K:1,Ta:2,Tb:5,Tc:8,L:0},
  "3rdZero":{K:1,Ta:5,Tb:9,Tc:12,Tz:11,L:1},
  "4th":{K:1,Ta:1,Tb:1,Tc:3,Td:5,L:0},
};
const SIM_COLORS=["#2563eb","#dc2626","#16a34a","#9333ea","#f59e0b","#06b6d4","#ec4899","#6366f1","#8b5cf6","#10b981"];

// ============================================================
// MAIN APP
// ============================================================
export default function App() {
  const [idType, setIdType] = useState("2nd");
  const [idParams, setIdParams] = useState(DEF_PARAMS);
  const [tau, setTau] = useState(0.15);
  const [tauTCM, setTauTCM] = useState(0.15);
  const [idResult, setIdResult] = useState(null);
  const [idChart, setIdChart] = useState([]);
  const [idStatus, setIdStatus] = useState("READY");

  // simConfig は同定結果 {K,T1,T2,L,tau} をそのまま受け取る
  const [simConfig, setSimConfig] = useState(null);
  const [history, setHistory] = useState([]);
  const [isRunning, setIsRunning] = useState(false);
  const [statusMsg, setStatusMsg] = useState("Waiting for identification...");
  const pidRef = useRef({ kp:0, ki:0, kd:0 });

  // --- 同定実行 ---
  const handleIdRun = useCallback(() => {
    setIdStatus("RUNNING");
    setIdResult(null);
    setIdChart([]);
    setHistory([]);
    setIsRunning(false);
    setStatusMsg("Waiting for identification...");
    try {
      const p = idParams[idType];
      const id = runId(idType, p, tau); // tau も runId 内で結果に含める
      if (!id) { setIdStatus("ERROR"); return; }

      // チャート用データ
      const dt_c = Math.max(tau, 0.05);
      const tS = idType==="2nd"||idType==="2ndZero" ? p.T1+p.T2+p.L
               : idType==="3rd"||idType==="3rdZero" ? p.Ta+p.Tb+p.Tc+p.L
               : p.Ta+p.Tb+p.Tc+p.Td+p.L;
      const dc = Math.min(15*tS, 250);
      const orig = idType==="2nd"     ? step2nd(p.K,p.T1,p.T2,p.L,dt_c,dc)
                 : idType==="2ndZero" ? step2ndZero(p.K,p.T1,p.T2,p.Tz,p.L,dt_c,dc)
                 : idType==="3rd"     ? step3rd(p.K,p.Ta,p.Tb,p.Tc,p.L,dt_c,dc)
                 : idType==="3rdZero" ? step3rdZero(p.K,p.Ta,p.Tb,p.Tc,p.Tz,p.L,dt_c,dc)
                 : step4th(p.K,p.Ta,p.Tb,p.Tc,p.Td,p.L,dt_c,dc);
      const repr = step2nd(id.K, id.T1, id.T2, id.L, dt_c, dc);
      const sk = Math.max(1, Math.round(2/dt_c));
      setIdChart(orig.filter((_,i)=>i%sk===0).map((pt,i)=>({
        t: +pt.t.toFixed(2),
        original: +pt.y.toFixed(5),
        identified: repr[i*sk] ? +repr[i*sk].y.toFixed(5) : null
      })));

      setIdResult(id);
      setIdStatus("DONE");

      // ★ 同定結果 K,T1,T2,L,tau,tauTCM をシミュレータへ転送
      // const cfg = { K: id.K, T1: id.T1, T2: id.T2, L: id.L, tau: id.tau, tauTCM };
      // ★ 同定結果を無次元化してシミュレータへ転送
        const Tbase = Math.max(id.T1, id.T2);  // T1 > T2 比較
        const cfg = {
          K:      id.K,
          T1:     id.T1     / Tbase,
          T2:     id.T2     / Tbase,
          L:      id.L      / Tbase,
          tau:    id.tau    / Tbase,
          tauTCM: tauTCM    / Tbase
        };
      setSimConfig(cfg);
      setStatusMsg("ID transferred — starting...");
      setTimeout(() => setIsRunning(true), 150);

    } catch(e) {
      setIdStatus("ERROR");
      console.error(e);
    }
  }, [idType, idParams, tau, tauTCM]);

  // --- シミュレータ本体 ---
  const runSimulation = useCallback((mode, kpIn, kiIn, kdIn) => {
    if (!simConfig) return null;
    const { K, T1, T2, L, tau: t, tauTCM: tTCM } = simConfig;
    const dt=0.01, dist=1.0;
    const tc = tTCM || t;
    const limit = Math.max(15.0, (T1+T2+L)*10);
    const ls=Math.max(1,Math.round(L/dt)), ts=Math.max(1,Math.round(tc/dt));
    let kp=kpIn, ki=kiIn, kd=kdIn;
    if (mode===0) {
      const ti=(T1*T2/(T1-T2))*Math.log(T1/T2);
      const rs=(Math.exp(-ti/T1)-Math.exp(-ti/T2))/(T1-T2);
      const yi=1-(T1*Math.exp(-ti/T1)-T2*Math.exp(-ti/T2))/(T1-T2);
      const lt=L+(ti-(yi/rs));
      const g0=rs*tc, l0=lt/tc;
      ki=0.6/(g0*Math.pow(l0+0.5,2));
      kp=(1.2/(g0*(l0+1)))-0.5*ki;
      kd=0.4/g0;
    }
    let y=0,y1=0,y2=0,mv=0,z1=0,z2=0,buf=new Array(ls+1).fill(0);
    const p1=Math.exp(-dt/T1), p2=Math.exp(-dt/T2);
    const b1=K*(1-(T1*p1-T2*p2)/(T1-T2)), b2=K*(p1*p2-(T1*p2-T2*p1)/(T1-T2));
    const points=[], fullResolutionE=[];
    for (let k=0; k<=Math.floor(limit/dt); k++) {
      if(k%ts===0){points.push({time:parseFloat((k*dt).toFixed(2)),y});mv+=kp*(y1-y)+ki*(0-y)+kd*(2*y1-y2-y);}
      const ny=(p1+p2)*z1-(p1*p2)*z2+b1*(buf[ls-1]+dist)+b2*(buf[ls]+dist);
      if(k%ts===0) fullResolutionE.push(-ny);
      for(let j=ls;j>0;j--)buf[j]=buf[j-1];
      buf[0]=mv;
      if(k%ts===0){y2=y1;y1=y;}
      y=ny;z2=z1;z1=ny;
    }
    return{params:{kp,ki,kd},points,fullResolutionE,tauTCM:tc};
  }, [simConfig]);

  const calculateAreas = useCallback((eArray) => {
    const dt=0.01;
    let areas=[0,0,0,0,0],cs=0,ai=0;
    for(let k=0;k<eArray.length-1;k++){
      const yk=eArray[k],yn=eArray[k+1],dA=(dt/2)*(yk+yn);
      if(yk*yn<0){if(ai<5){areas[ai]=cs+dA*0.5;ai++;cs=dA*0.5;continue;}else break;}
      cs+=dA;
    }
    if(ai<5)areas[ai]=cs;
    const ap1=Math.abs(areas[0]),am1=Math.abs(areas[1]),ap2=Math.abs(areas[2]),am2=Math.abs(areas[3]),ap3=Math.abs(areas[4]);
    if(ap1<1e-18)return{a1:0,a2:0,a3:0,a4:0,a5:0};
    const a1=(am1/ap1)*100,a2=(ap2/ap1)*100,a3=am1>1e-15?(am2/am1)*100:0,a4=am1>1e-15?(ap2/am1)*100:0,a5=(ap3/ap1)*100;
    return{a1,a2,a3:(a3>=20000||am1<1e-12)?-99999:a3,a4:(a4>=100000||am1<1e-12)?-99999:a4,a5};
  }, []);

  const runFuzzyInference = useCallback((eArray) => {
    const alphas=calculateAreas(eArray);
    const d1=rT1(alphas.a4,alphas.a5,ALPHA_TABLES.alpha5,ALPHA_TABLES.alpha4,membership_alpha4,membership_kd1_table57,OUT_KD_TYPE1);
    if(d1>0)return{target:'kd',delta:d1,rule:1,alphas};
    const d2=rT2(alphas.a3,ALPHA_TABLES.alpha3,membership_alpha3,membership_kd2_table511,OUT_KD_TYPE2);
    if(d2>0)return{target:'kd',delta:d2,rule:2,alphas};
    if(alphas.a2>=alphas.a1){
      const d3=rT3(alphas.a2,ALPHA_TABLES.alpha2,membership_alpha2,membership_delta_kp,out_kp_universe);
      if(d3>0)return{target:'kp',delta:d3,rule:3,alphas};
    }else{
      const d4=rT4(alphas.a1,ALPHA_TABLES.alpha1,membership_alpha1,membership_delta_ki,out_ki_universe);
      if(d4>0)return{target:'ki',delta:d4,rule:4,alphas};
    }
    return{target:'stable',delta:0,rule:0,alphas};
  }, [calculateAreas]);

  const executeNextStep = useCallback(() => {
    setHistory(prev => {
      const si=prev.length;
      if(si>=21){setIsRunning(false);return prev;}
      let res,debug;
      if(si===0){
        res=runSimulation(0,0,0,0);
        if(!res){setIsRunning(false);return prev;}
        pidRef.current={...res.params};
        debug={alphas:{a1:0,a2:0,a3:0,a4:0,a5:0},target:"initial",delta:0,rule:"Takahashi-Chan (initial)"};
        setStatusMsg("Initial Tuning Applied");
      }else{
        const last=prev[si-1];
        const infer=runFuzzyInference(last.fullResolutionE);
        debug=infer;
        if(infer.target==='kp')pidRef.current.kp*=(1-infer.delta/100);
        if(infer.target==='ki')pidRef.current.ki*=(1-infer.delta/100);
        if(infer.target==='kd')pidRef.current.kd*=(1-infer.delta/100);
        if(infer.target==='stable'){setIsRunning(false);setStatusMsg("Converged.");}
        else setStatusMsg(`Adjusting ${infer.target.toUpperCase()}`);
        res=runSimulation(1,pidRef.current.kp,pidRef.current.ki,pidRef.current.kd);
        if(!res){setIsRunning(false);return prev;}
      }
      return[...prev,{...res,debug,color:SIM_COLORS[si%10]}];
    });
  }, [runSimulation, runFuzzyInference]);

  useEffect(()=>{
    if(!isRunning)return;
    const t=setTimeout(executeNextStep,600);
    return()=>clearTimeout(t);
  },[isRunning,history,executeNextStep]);

  const curP = idParams[idType];
  const scColor = idStatus==="DONE"?"#3fb950":idStatus==="ERROR"?"#f85149":idStatus==="RUNNING"?"#e3b341":"#8b949e";

  return (
    <div style={{minHeight:"100vh",background:"#0d1117",color:"#e6edf3",fontFamily:"'Courier New',monospace",padding:12,boxSizing:"border-box"}}>
      {/* HEADER */}
      <div style={{display:"flex",justifyContent:"space-between",alignItems:"center",borderBottom:"1px solid #30363d",paddingBottom:10,marginBottom:12,flexWrap:"wrap",gap:8}}>
        <div>
          <div style={{fontSize:9,color:"#8b949e",letterSpacing:"0.2em",marginBottom:2}}>KANAGAWA 1987 → 2026  INTEGRATED SYSTEM</div>
          <h1 style={{fontSize:18,fontWeight:900,margin:0}}>PROCESS ID <span style={{color:"#58a6ff"}}>+</span> I-PD FUZZY ENGINE <span style={{color:"#58a6ff"}}>v5.4</span></h1>
        </div>
        <div style={{fontSize:11,fontWeight:700,color:scColor,padding:"4px 12px",border:`1px solid ${scColor}`,borderRadius:6}}>
          ID: {idStatus}　　SIM: {isRunning?"RUNNING":statusMsg}
        </div>
      </div>

      <div style={{display:"grid",gridTemplateColumns:"250px 1fr 330px",gap:12,height:"calc(100vh - 90px)"}}>

        {/* LEFT: IDENTIFICATION */}
        <div style={{display:"flex",flexDirection:"column",gap:8,overflowY:"auto"}}>
          <div style={{fontSize:9,color:"#8b949e",fontWeight:700,letterSpacing:"0.15em"}}>① PROCESS IDENTIFICATION</div>
          <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:3}}>
            {TYPE_LIST.map(([t,lbl])=>(
              <button key={t} onClick={()=>setIdType(t)}
                style={{background:idType===t?"#58a6ff":"transparent",color:idType===t?"#0d1117":"#8b949e",border:`1px solid ${idType===t?"#58a6ff":"#30363d"}`,borderRadius:5,padding:"4px 0",fontFamily:"monospace",fontSize:9,fontWeight:700,cursor:"pointer"}}>
                {lbl}
              </button>
            ))}
          </div>

          <div style={{background:"#161b22",border:"1px solid #30363d",borderRadius:8,padding:10}}>
            <div style={{fontSize:9,color:"#8b949e",letterSpacing:"0.15em",marginBottom:8,fontWeight:700}}>PROCESS PARAMETERS</div>
            {FIELDS[idType].map(([key,label,step,isZero])=>(
              <div key={key} style={{marginBottom:5}}>
                <div style={{fontSize:9,color:isZero?"#e3b341":"#8b949e",marginBottom:2}}>{label}{isZero?" ★zero":""}</div>
                <input type="number" step={step} value={curP[key]??0}
                  onChange={e=>setIdParams(p=>({...p,[idType]:{...p[idType],[key]:parseFloat(e.target.value)||0}}))}
                  style={{width:"100%",background:"#0d1117",border:`1px solid ${isZero?"#e3b341":"#30363d"}`,borderRadius:5,color:"#58a6ff",fontFamily:"monospace",fontSize:14,fontWeight:700,padding:"4px 7px",outline:"none",boxSizing:"border-box"}}
                />
              </div>
            ))}
            <div style={{borderTop:"1px solid #30363d",marginTop:8,paddingTop:8}}>
              <div style={{fontSize:9,color:"#58a6ff",marginBottom:2}}>τ(ID) — 同定用サンプリング周期</div>
              <input type="number" step={0.05} value={tau}
                onChange={e=>setTau(parseFloat(e.target.value)||0.01)}
                style={{width:"100%",background:"#0d1117",border:"1px solid #58a6ff",borderRadius:5,color:"#58a6ff",fontFamily:"monospace",fontSize:14,fontWeight:700,padding:"4px 7px",outline:"none",boxSizing:"border-box"}}
              />
            </div>
            <div style={{marginTop:6}}>
              <div style={{fontSize:9,color:"#e3b341",marginBottom:2}}>τ(TCM) — TCM/制御用サンプリング周期</div>
              <input type="number" step={0.05} value={tauTCM}
                onChange={e=>{const v=parseFloat(e.target.value);if(!isNaN(v)&&v>0)setTauTCM(v);}}
                style={{width:"100%",background:"#0d1117",border:"1px solid #e3b341",borderRadius:5,color:"#e3b341",fontFamily:"monospace",fontSize:14,fontWeight:700,padding:"4px 7px",outline:"none",boxSizing:"border-box"}}
              />
            </div>
          </div>

          <button onClick={handleIdRun} disabled={idStatus==="RUNNING"||isRunning}
            style={{background:idStatus==="RUNNING"||isRunning?"#30363d":"#58a6ff",color:idStatus==="RUNNING"||isRunning?"#8b949e":"#0d1117",border:"none",borderRadius:7,padding:"8px 0",fontFamily:"monospace",fontSize:12,fontWeight:900,cursor:idStatus==="RUNNING"||isRunning?"not-allowed":"pointer"}}>
            {idStatus==="RUNNING"?"COMPUTING...":"▶  IDENTIFY & RUN SIM"}
          </button>
          <button onClick={()=>{setIdResult(null);setIdChart([]);setIdStatus("READY");setHistory([]);setIsRunning(false);setSimConfig(null);setStatusMsg("Waiting for identification...");}}
            style={{background:"transparent",color:"#8b949e",border:"1px solid #30363d",borderRadius:6,padding:"4px 0",fontFamily:"monospace",fontSize:10,cursor:"pointer"}}>
            RESET ALL
          </button>

          {idResult&&(
            <div style={{background:"#161b22",border:"1px solid #3fb950",borderRadius:8,padding:10,fontSize:18}}>
              <div style={{color:"#3fb950",fontWeight:700,letterSpacing:"0.15em",marginBottom:6,fontSize:18}}>IDENTIFIED G_M(s)</div>
              <div style={{color:"#8b949e",lineHeight:1.9,marginBottom:6,fontSize:18}}>
                = {idResult.K.toFixed(6)}·e^(-{idResult.L.toFixed(6)}s)<br/>
                　/(({idResult.T1.toFixed(6)}s+1)<br/>
                　({idResult.T2.toFixed(6)}s+1))
              </div>
              <div style={{color:"#e6edf3",lineHeight:1.8,fontSize:18}}>
                α={idResult.alpha.toFixed(7)}<br/>
                β={idResult.beta.toFixed(7)}<br/>
                Tm={idResult.Tm.toFixed(7)}
              </div>
              <div style={{marginTop:6,padding:"4px 6px",background:"#0d1117",borderRadius:4,color:"#3fb950",fontSize:18,fontWeight:700}}>
                → Sim: K={idResult.K.toFixed(6)}, T1={idResult.T1.toFixed(6)}, T2={idResult.T2.toFixed(6)}, L={idResult.L.toFixed(6)}, τID={tau}, τTCM={tauTCM}
              </div>
            </div>
          )}
        </div>

        {/* CENTER: CHARTS */}
        <div style={{display:"flex",flexDirection:"column",gap:10,minWidth:0}}>
          <div style={{background:"#161b22",border:"1px solid #30363d",borderRadius:8,padding:12,flex:"0 0 auto",height:200}}>
            <div style={{fontSize:9,color:"#8b949e",fontWeight:700,marginBottom:6}}>
              STEP RESPONSE　<span style={{color:"#58a6ff"}}>─── Original</span>　<span style={{color:"#ffa657"}}>─ ─ G_M(s)</span>
            </div>
            {idChart.length>0?(
              <ResponsiveContainer width="100%" height="82%">
                <LineChart data={idChart} margin={{top:2,right:8,left:-15,bottom:2}}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#21262d" vertical={false}/>
                  <XAxis dataKey="t" stroke="#8b949e" fontSize={9} tick={{fill:"#e6edf3"}}/>
                  <YAxis stroke="#8b949e" fontSize={9} tick={{fill:"#e6edf3"}} domain={[0,"auto"]}/>
                  <Tooltip contentStyle={{background:"#161b22",border:"1px solid #30363d",borderRadius:6,fontSize:9,color:"#e6edf3"}}/>
                  <Line type="monotone" dataKey="original"   stroke="#58a6ff" dot={false} strokeWidth={2} isAnimationActive={false}/>
                  <Line type="monotone" dataKey="identified" stroke="#ffa657" dot={false} strokeWidth={1.5} strokeDasharray="5 3" isAnimationActive={false}/>
                </LineChart>
              </ResponsiveContainer>
            ):(
              <div style={{height:"82%",display:"flex",alignItems:"center",justifyContent:"center",color:"#8b949e",fontSize:10}}>
                ▶ IDENTIFY &amp; RUN SIM を押してください
              </div>
            )}
          </div>

          <div style={{background:"white",borderRadius:"16px",padding:"14px 18px",flex:1,display:"flex",flexDirection:"column",minHeight:0}}>
            <div style={{display:"flex",justifyContent:"space-between",alignItems:"center",marginBottom:8}}>
              <span style={{fontSize:13,fontWeight:900,color:"#cbd5e1",textTransform:"uppercase",letterSpacing:"0.12em",fontStyle:"italic"}}>I-PD Integral Precise Plot</span>
              {simConfig&&(
                <span style={{fontSize:14,fontFamily:"monospace",color:"#94a3b8"}}>
                  K={simConfig.K.toFixed(3)}　T1={simConfig.T1.toFixed(3)}　T2={simConfig.T2.toFixed(3)}　L={simConfig.L.toFixed(3)}　τID={simConfig.tau}　τTCM={simConfig.tauTCM}
                </span>
              )}
            </div>
            <div style={{flex:1,minHeight:0}}>
              <ResponsiveContainer width="100%" height="100%">
                <LineChart margin={{top:6,right:8,left:-20,bottom:0}}>
                  <CartesianGrid strokeDasharray="3 3" vertical={false} stroke="#f1f5f9"/>
                  <XAxis dataKey="time" type="number" domain={[0, simConfig ? Math.max(15, (simConfig.T1+simConfig.T2+simConfig.L)*10) : 15]} fontSize={10} stroke="#94a3b8"/>
                  <YAxis domain={["auto","auto"]} fontSize={10} stroke="#94a3b8"/>
                  <Tooltip contentStyle={{borderRadius:"12px",border:"none",boxShadow:"0 8px 20px rgba(0,0,0,0.1)",fontSize:10}}/>
                  <ReferenceLine y={0} stroke="#cbd5e1" strokeWidth={1.5}/>
                  {history.map((h,i)=>(
                    <Line key={i} data={h.points} type="monotone" dataKey="y" stroke={h.color} dot={false}
                      strokeWidth={i===history.length-1?3.5:1.5} opacity={i===history.length-1?1:0.2} isAnimationActive={false}/>
                  ))}
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>

        {/* RIGHT: LOG */}
        <div style={{background:"#0f172a",borderRadius:"16px",padding:"16px",color:"white",display:"flex",flexDirection:"column",overflow:"hidden"}}>
          <div style={{display:"flex",justifyContent:"space-between",alignItems:"center",borderBottom:"1px solid rgba(255,255,255,0.1)",paddingBottom:10,marginBottom:12}}>
            <span style={{fontSize:10,fontWeight:900,textTransform:"uppercase",letterSpacing:"0.12em"}}>② I-PD Engine Log</span>
            <span style={{fontSize:18,color:"#60a5fa",fontFamily:"monospace",fontWeight:700}}>{history.length} Steps</span>
          </div>
          <div style={{flex:1,overflowY:"auto",display:"flex",flexDirection:"column",gap:10,paddingRight:2}}>
            {history.length===0&&(
              <div style={{color:"rgba(255,255,255,0.2)",fontSize:10,textAlign:"center",marginTop:20}}>
                同定完了後に自動開始されます
              </div>
            )}
            {[...history].reverse().map((h,i)=>(
              <div key={i} style={{padding:"12px",borderRadius:"12px",border:`1px solid ${i===0?"rgba(255,255,255,0.2)":"rgba(255,255,255,0.07)"}`,background:i===0?"rgba(255,255,255,0.09)":"rgba(255,255,255,0.03)",opacity:i===0?1:0.7}}>
                <div style={{display:"flex",justifyContent:"space-between",alignItems:"flex-start",marginBottom:8}}>
                  <div>
                    <div style={{fontSize:26,fontWeight:900,color:h.color,display:"flex",alignItems:"center",gap:5}}>
                      <span style={{width:7,height:7,borderRadius:"50%",background:h.color,display:"inline-block"}}/>
                      STEP #{history.length-i-1}
                    </div>
                    <div style={{fontSize:24,fontWeight:900,fontStyle:"italic",marginTop:2,color:h.debug.target==="stable"?"#4ade80":"#fb923c"}}>
                      {h.debug.target==="stable"?"✓ STABLE":`ADJUST ${String(h.debug.target).toUpperCase()} (-${Number(h.debug.delta).toFixed(1)}%)`}
                    </div>
                  </div>
                  <span style={{fontSize:8,fontFamily:"monospace",color:"rgba(255,255,255,0.25)",padding:"2px 6px",background:"rgba(255,255,255,0.04)",borderRadius:6}}>Rule:{h.debug.rule}</span>
                </div>
                <div style={{background:"rgba(0,0,0,0.35)",borderRadius:8,padding:8,marginBottom:8}}>
                  <div style={{fontSize:10,color:"rgba(255,255,255,0.25)",marginBottom:5,textTransform:"uppercase"}}>Ratios a1–a5</div>
                  <div style={{display:"grid",gridTemplateColumns:"repeat(5,1fr)",gap:3}}>
                    {Object.entries(h.debug.alphas).map(([key,val])=>(
                      <div key={key} style={{display:"flex",flexDirection:"column",alignItems:"center"}}>
                        <span style={{fontSize:13,color:"rgba(255,255,255,0.6)",fontWeight:700,fontStyle:"italic"}}>{key}</span>
                        <div style={{width:"100%",height:2,background:"rgba(255,255,255,0.07)",borderRadius:2,overflow:"hidden",margin:"2px 0"}}>
                          <div style={{height:"100%",background:"#3b82f6",width:`${Math.min(100,Math.abs(val))}%`}}/>
                        </div>
                        <span style={{fontSize:13,fontFamily:"monospace",fontWeight:700,color:val<0?"#f87171":"rgba(255,255,255,0.85)"}}>
                          {val<-90000?"N/A":val.toFixed(1)}
                        </span>
                      </div>
                    ))}
                  </div>
                </div>
                <div style={{display:"grid",gridTemplateColumns:"repeat(3,1fr)",gap:4,textAlign:"center",fontFamily:"monospace"}}>
                  {[["Kp",h.params.kp,"#bfdbfe"],["Ki",h.params.ki,"#bbf7d0"],["Kd",h.params.kd,"#fecaca"]].map(([label,val,clr])=>(
                    <div key={label} style={{background:"rgba(0,0,0,0.2)",padding:"6px 4px",borderRadius:7,border:"1px solid rgba(255,255,255,0.04)"}}>
                      <div style={{fontSize:12,color:"rgba(255,255,255,0.4)",textTransform:"uppercase",marginBottom:1}}>{label}</div>
                      <div style={{fontSize:16,fontWeight:700,color:clr}}>{val.toFixed(3)}</div>
                    </div>
                  ))}
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}