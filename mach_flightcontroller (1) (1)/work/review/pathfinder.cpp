#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdint>
extern "C" __declspec(dllexport) int findpath(int nx,int ny,const uint8_t* blocked,const float* costs,int start,const int* targets,int nt,int* output){
 int nn=nx*ny,xmin=nx,xmax=0,ymin=ny,ymax=0; std::vector<uint8_t> goal(nn,0),closed(nn,0);std::vector<float>d(nn,1e30f);std::vector<int>prev(nn,-1);
 for(int i=0;i<nt;i++){int p=targets[i];goal[p]=1;xmin=std::min(xmin,p%nx);xmax=std::max(xmax,p%nx);ymin=std::min(ymin,p/nx);ymax=std::max(ymax,p/nx);}
 auto h=[&](int x,int y){int dx=std::max({xmin-x,0,x-xmax}),dy=std::max({ymin-y,0,y-ymax});return float(std::max(dx,dy)+.41421356*std::min(dx,dy));};
 using E=std::pair<float,int>;std::priority_queue<E,std::vector<E>,std::greater<E>>q;d[start]=0;q.push({h(start%nx,start/nx),start});
 const int dxs[]={1,-1,0,0,1,1,-1,-1},dys[]={0,0,1,-1,1,-1,1,-1};
 while(!q.empty()){int c=q.top().second;q.pop();if(closed[c])continue;if(goal[c]){int count=0;while(c!=start&&count<nn){output[count++]=c;c=prev[c];}output[count++]=start;std::reverse(output,output+count);return count;}closed[c]=1;int x=c%nx,y=c/nx;
 for(int i=0;i<8;i++){int xx=x+dxs[i],yy=y+dys[i];if(xx<0||xx>=nx||yy<0||yy>=ny)continue;int v=yy*nx+xx;if(blocked[v]&&!goal[v])continue;if(i>=4&&(blocked[y*nx+xx]||blocked[yy*nx+x]))continue;float nd=d[c]+(i<4?1.f:1.41421356f)*(1.f+costs[v]);if(nd<d[v]){d[v]=nd;prev[v]=c;q.push({nd+1.02f*h(xx,yy),v});}}
 }
 return 0;
}
