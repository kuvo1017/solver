#include "autogl_mates.h"
#include <stdio.h>
#include <autogl.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

/*======================================================================*/
void AutoGL_DrawBoldLine2D(
    double x0, double y0, double z0,
    double x1, double y1, double z1,
    double width)
{
    /* 方向ベクトル */
    double v[3];
    /* 法線ベクトル(z軸に平行な直線だと仮定している。*/
    double l;
    double nv[3];

    v[0] = x1 - x0;
    v[1] = y1 - y0;
    v[2] = z1 - z0;

    l =  sqrt(v[0]*v[0] + v[1]*v[1]) ;
    nv[0] = -v[1] / l;
    nv[1] = v[0] / l;
    nv[2] = 0;

    AutoGL_DrawQuadrangle(x0 + nv[0] * width, y0 + nv[1] * width, z0,
                          x0 - nv[0] * width, y0 - nv[1] * width, z0,
                          x1 - nv[0] * width, y1 - nv[1] * width, z1,
                          x1 + nv[0] * width, y1 + nv[1] * width, z1);
}

/*======================================================================*/
void AutoGL_DrawArrow2D(
    double x0, double y0, double z0,
    double x1, double y1, double z1)
{
    /* 方向ベクトル */
    double v[3];
    /* 法線ベクトル(z軸に平行な直線だと仮定している。*/
    double l;
    double nv[3];

    /* 矢印の先端は３角形にする。この３角形を書くための３点を導出 */
    double p0[3], p1[3], p2[3];

    v[0] = x1 - x0;
    v[1] = y1 - y0;
    v[2] = z1 - z0;

    l =  sqrt(v[0]*v[0] + v[1]*v[1]) ;
    nv[0] = -v[1] / l;
    nv[1] = v[0] / l;
    nv[2] = 0; 
  
    p0[0] = x1;
    p0[1] = y1;
    p0[2] = z1;
    p1[0] =  0;
    p1[1] =  0;
    p1[2] =  0;
    p2[0] =  0;
    p2[1] =  0;
    p2[2] =  0;
  
    l = l /2;
    if(l > 1)
    {
        p1[0] = (p0[0] - v[0] / l ) + nv[0]/2;
        p1[1] = (p0[1] - v[1] / l ) + nv[1]/2;
        p2[0] = (p0[0] - v[0] / l ) - nv[0]/2;
        p2[1] = (p0[1] - v[1] / l ) - nv[1]/2;
    }
    else
    {
        p1[0] = (p0[0] - v[0] / 2 ) + nv[0]/2;
        p1[1] = (p0[1] - v[1] / 2 ) + nv[1]/2;
        p2[0] = (p0[0] - v[0] / 2 ) - nv[0]/2;
        p2[1] = (p0[1] - v[1] / 2 ) - nv[1]/2;
    }

    AutoGL_DrawLine( x0,  y0,  z0,
                     x1,  y1,  z1);
    AutoGL_DrawTriangle( p0[0], p0[1], p0[2],
                         p1[0], p1[1], p1[2],
                         p2[0], p2[1], p2[2]);
}

/*======================================================================*/
void AutoGL_DrawBoldArrow2D
(double width,
 double x0, double y0, double z0,
 double x1, double y1, double z1){
    /* 方向ベクトル */
    double v[3];
    /* 法線ベクトル(z軸に平行な直線だと仮定している。*/
    double l;
    double nv[3];

    /* 矢印の先端は３角形にする。この３角形を書くための３点を導出 */
    double p0[3], p1[3], p2[3];

    v[0] = x1 - x0;
    v[1] = y1 - y0;
    v[2] = z1 - z0;

    l =  sqrt(v[0]*v[0] + v[1]*v[1]) ;
    nv[0] = -v[1] / l;
    nv[1] = v[0] / l;
    nv[2] = 0; 
  
    p0[0] = x1;
    p0[1] = y1;
    p0[2] = z1;
    p1[0] =  0;
    p1[1] =  0;
    p1[2] =  0;
    p2[0] =  0;
    p2[1] =  0;
    p2[2] =  0;
  
    l = l /2;
    if(l > 1)
    {
        p1[0] = (p0[0] - v[0] / l ) + nv[0]/2;
        p1[1] = (p0[1] - v[1] / l ) + nv[1]/2;
        p2[0] = (p0[0] - v[0] / l ) - nv[0]/2;
        p2[1] = (p0[1] - v[1] / l ) - nv[1]/2;
    }
    else
    {
        p1[0] = (p0[0] - v[0] / 2 ) + nv[0]/2;
        p1[1] = (p0[1] - v[1] / 2 ) + nv[1]/2;
        p2[0] = (p0[0] - v[0] / 2 ) - nv[0]/2;
        p2[1] = (p0[1] - v[1] / 2 ) - nv[1]/2;
    }

    p1[2] = p2[2] = p0[2];

    AutoGL_DrawTriangle( p0[0], p0[1], p0[2],
                         p1[0], p1[1], p1[2],
                         p2[0], p2[1], p2[2]);

    AutoGL_DrawBoldLine2D(x0, y0, z0,
                          x1, y1, z1,
                          width);
}

/*======================================================================*/
void AutoGL_DrawStickyString(
    int sizeDc,
    const char *str)
{
    int widthDc, heightDc;
    int centerXDc, centerYDc;
    double centerX, centerY, centerZ;
  
    AutoGL_GetViewRangeDc (&widthDc, &heightDc);
  
    centerXDc = -widthDc / 2 + sizeDc;
    centerYDc = heightDc/2 - sizeDc;
    AutoGL_GetPositionOfDc (&centerX, &centerY, &centerZ,
                            centerXDc, centerYDc, 0);
    centerZ += 0.1;
    AutoGL_DrawString(centerX, centerY, centerZ, str);
}

/*======================================================================*/
void AutoGL_DrawSticyQuadrangle2D(
    int margin, double xWidth, double yWidth)
{
    int widthDc, heightDc;
    int centerXDc, centerYDc;
    double centerX, centerY, centerZ;
  
    AutoGL_GetViewRangeDc (&widthDc, &heightDc);
  
    centerXDc = -widthDc / 2 + margin;
    centerYDc = -heightDc / 2 + margin;
    AutoGL_GetPositionOfDc (&centerX, &centerY, &centerZ,
                            centerXDc, centerYDc, 0);
    xWidth = xWidth * AutoGL_GetViewSize () / AutoGL_GetViewSizeDc ();
    yWidth = yWidth * AutoGL_GetViewSize () / AutoGL_GetViewSizeDc ();
    AutoGL_DrawQuadrangle(centerX-xWidth/2, centerY-yWidth/2,centerZ+100,
                          centerX+xWidth/2, centerY-yWidth/2,centerZ+100,
                          centerX+xWidth/2, centerY+yWidth/2,centerZ+100,
                          centerX-xWidth/2, centerY+yWidth/2,centerZ+100);
}

/*======================================================================*/
void AutoGL_SetContourMap_RB()
{
    AutoGL_ClearContourColor();
    AutoGL_AddContourColorOfGrade(0.0, 
                                  1.0, 0.0, 0.0);
    AutoGL_AddContourColorOfGrade(1.0, 
                                  0.0, 0.0, 1.0);
}

/*======================================================================*/
void AutoGL_DrawBackground2D()
{
    int widthDc, heightDc;
    int LeftTopXDc, LeftTopYDc;
    int RightBottomXDc, RightBottomYDc;
    double LeftTopX, LeftTopY, LeftTopZ;
    double RightBottomX, RightBottomY, RightBottomZ;
  
    AutoGL_GetViewRangeDc (&widthDc, &heightDc);
  
    LeftTopXDc = -widthDc;
    LeftTopYDc = heightDc;
    RightBottomXDc = widthDc;
    RightBottomYDc = -heightDc;
  
    AutoGL_GetPositionOfDc (&LeftTopX, &LeftTopY, &LeftTopZ,
                            LeftTopXDc, LeftTopYDc, -10);
    AutoGL_GetPositionOfDc (&RightBottomX, &RightBottomY, &RightBottomZ,
                            RightBottomXDc, RightBottomYDc, -10);
    AutoGL_DrawQuadrangle(LeftTopX, LeftTopY, -10,
                          LeftTopX, RightBottomY, -10,
                          RightBottomX, RightBottomY, -10,
                          RightBottomX, LeftTopY, -10);
}

/*======================================================================*/
void AutoGL_DrawCircle2D
(double x0, double y0, double radius)
{
  int nDivide = 8;
  double point[nDivide][2]; 
  int i;
  for(i=0;i<nDivide;i++)
  {
    point[i][0] = radius * cos(2*M_PI/nDivide * (double) i);
    point[i][1] = radius * sin(2*M_PI/nDivide * (double) i);
  }
  
  for(i=0;i<nDivide;i++)
  {
    if(i != nDivide-1)
    {
      AutoGL_DrawTriangle(x0,y0,0,
          point[i][0],point[i][1],0,
          point[i+1][0],point[i+1][1],0);
    }else
    {
      AutoGL_DrawTriangle(x0,y0,0,
          point[i][0],point[i][1],0,
          point[0][0],point[0][1],0);
    }
  }
}

