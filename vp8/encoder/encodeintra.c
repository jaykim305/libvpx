/*
 *  Copyright (c) 2010 The WebM project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "vpx_config.h"
#include "vp8_rtcd.h"
#include "./vpx_dsp_rtcd.h"
#include "vp8/encoder/quantize.h"
#include "vp8/common/reconintra.h"
#include "vp8/common/reconintra4x4.h"
#include "encodemb.h"
#include "vp8/common/invtrans.h"
#include "encodeintra.h"

int vp8_encode_intra(MACROBLOCK *x, int use_dc_pred) {
  int i;
  int intra_pred_var = 0;

  if (use_dc_pred) {
    x->e_mbd.mode_info_context->mbmi.mode = DC_PRED;
    x->e_mbd.mode_info_context->mbmi.uv_mode = DC_PRED;
    x->e_mbd.mode_info_context->mbmi.ref_frame = INTRA_FRAME;

    vp8_encode_intra16x16mby(x);

    vp8_inverse_transform_mby(&x->e_mbd);
  } else {
    for (i = 0; i < 16; ++i) {
      x->e_mbd.block[i].bmi.as_mode = B_DC_PRED;
      vp8_encode_intra4x4block(x, i);
    }
  }

  intra_pred_var = vpx_get_mb_ss(x->src_diff);

  return intra_pred_var;
}

void vp8_encode_intra4x4block(MACROBLOCK *x, int ib) {
  BLOCKD *b = &x->e_mbd.block[ib];
  BLOCK *be = &x->block[ib];
  int dst_stride = x->e_mbd.dst.y_stride;
  unsigned char *dst = x->e_mbd.dst.y_buffer + b->offset;
  unsigned char *Above = dst - dst_stride;
  unsigned char *yleft = dst - 1;
  unsigned char top_left = Above[-1];

  vp8_intra4x4_predict(Above, yleft, dst_stride, b->bmi.as_mode, b->predictor,
                       16, top_left);

  vp8_subtract_b(be, b, 16);

  x->short_fdct4x4(be->src_diff, be->coeff, 32);

  x->quantize_b(be, b);

  if (*b->eob > 1) {
    vp8_short_idct4x4llm(b->dqcoeff, b->predictor, 16, dst, dst_stride);
  } else {
    vp8_dc_only_idct_add(b->dqcoeff[0], b->predictor, 16, dst, dst_stride);
  }
}

void vp8_encode_intra4x4mby(MACROBLOCK *mb) {
  int i;

  MACROBLOCKD *xd = &mb->e_mbd;
  intra_prediction_down_copy(xd, xd->dst.y_buffer - xd->dst.y_stride + 16);

  for (i = 0; i < 16; ++i) vp8_encode_intra4x4block(mb, i);
  return;
}

void vp8_encode_intra16x16mby(MACROBLOCK *x) {
  BLOCK *b = &x->block[0];
  MACROBLOCKD *xd = &x->e_mbd;

  vp8_build_intra_predictors_mby_s(xd, xd->dst.y_buffer - xd->dst.y_stride,
                                   xd->dst.y_buffer - 1, xd->dst.y_stride,
                                   xd->dst.y_buffer, xd->dst.y_stride);

  vp8_subtract_mby(x->src_diff, *(b->base_src), b->src_stride, xd->dst.y_buffer,
                   xd->dst.y_stride);

  vp8_transform_intra_mby(x);

  vp8_quantize_mby(x);

  if (x->optimize) vp8_optimize_mby(x);
}

void vp8_encode_intra16x16mbuv(MACROBLOCK *x) {
  MACROBLOCKD *xd = &x->e_mbd;
  vp8_build_intra_predictors_mbuv_s(xd, xd->dst.u_buffer - xd->dst.uv_stride,
                                    xd->dst.v_buffer - xd->dst.uv_stride,
                                    xd->dst.u_buffer - 1, xd->dst.v_buffer - 1,
                                    xd->dst.uv_stride, xd->dst.u_buffer,
                                    xd->dst.v_buffer, xd->dst.uv_stride);

  vp8_subtract_mbuv(x->src_diff, x->src.u_buffer, x->src.v_buffer,
                    x->src.uv_stride, xd->dst.u_buffer, xd->dst.v_buffer,
                    xd->dst.uv_stride);

  vp8_transform_mbuv(x);

  vp8_quantize_mbuv(x);

  if (x->optimize) vp8_optimize_mbuv(x);
}


void vp8_encode_intra16x16mbuv_my(MACROBLOCK *x) {
  MACROBLOCKD *xd = &x->e_mbd;

  unsigned char *uptr_tracking = x->src.u_buffer;
  unsigned char *vptr_tracking = x->src.v_buffer;

  int r, c;
  uint16_t depth = 0;
  double depx = 0;
  int u, v;
  double u_, v_;
  unsigned char new_u, new_v;
  unsigned char min_uv = 16;
  unsigned char max_uv = 240;

#if 1
  
  for (r = 0; r < 8; ++r) {
    for (c = 0; c < 8; c++){
      u = (int) uptr_tracking[c];
      v = (int) vptr_tracking[c];
      
      // assert(u >= min_uv && u <= 255);
      // assert(v >= min_uv && v <= 255);
      // depth = u * 256 + v;
      u -= min_uv;
      v -= min_uv;

      depth = u * (max_uv-min_uv) + v;

      depx = (double) depth / 15000.;
      if (0 <= depx && depx < 0.25) u_ = 1./0.25 * depx;
      else if (0.25 <= depx && depx < 0.5) u_ = 1.;
      else if (0.5 <= depx && depx < 0.75) u_ = 1. - (depx - 0.5) / (0.75-0.5);
      else u_ = 0;

      if (0.25 <= depx && depx < 0.5) v_ = (depx - 0.25) / (0.5-0.25);
      else if (0.5 <= depx && depx < 0.75) v_ = 1.;
      else if (0.75 <= depx && depx < 1.) v_ = 1. - (depx - 0.75) / (1. - 0.75);
      else v_ = 0;

      // if (0 <= depx < 0.25) u_ = 1./0.25 * depx;
      // else if (0.25 <= depx < 0.5) u_ = 1.;
      // else if (0.5 <= depx < 0.75) u_ = 1. - (depx - 0.5) / (0.75-0.5);
      // else u_ = 0;

      // if (0.25 <= depx < 0.5) v_ = (depx - 0.25) / (0.5-0.25);
      // else if (0.5 <= depx < 0.75) v_ = 1.;
      // else if (0.75 <= depx < 1.) v_ = 1. - (depx - 0.75) / (1. - 0.75);
      // else v_ = 0;


      if (u_ > 1) u_ = 1;
      if (v_ > 1) v_ = 1;
      if (u_ < 0) u_ = 0;
      if (v_ < 0) v_ = 0;
      // assert(depth <= 15000);
      // if (depth) printf("restored depth %d\n", depth);
      new_u = (unsigned char)(u_ * (max_uv-min_uv) + min_uv); 
      new_v = (unsigned char)(v_ * (max_uv-min_uv) + min_uv);

      assert(new_u >= min_uv && new_u <= max_uv);
      assert(new_v >= min_uv && new_v <= max_uv);
      uptr_tracking[c] = new_u ; //255.0; //126; //(unsigned char)depth;
      vptr_tracking[c] = new_v; //255.0; //126; //(unsigned char)depth;
    }
    uptr_tracking += x->src.uv_stride;
    vptr_tracking += x->src.uv_stride;
  }
#endif

  vp8_build_intra_predictors_mbuv_s(xd, xd->dst.u_buffer - xd->dst.uv_stride,
                                    xd->dst.v_buffer - xd->dst.uv_stride,
                                    xd->dst.u_buffer - 1, xd->dst.v_buffer - 1,
                                    xd->dst.uv_stride, xd->dst.u_buffer,
                                    xd->dst.v_buffer, xd->dst.uv_stride);

  vp8_subtract_mbuv(x->src_diff, x->src.u_buffer, x->src.v_buffer,
                    x->src.uv_stride, xd->dst.u_buffer, xd->dst.v_buffer,
                    xd->dst.uv_stride);

  vp8_transform_mbuv(x);

  vp8_quantize_mbuv(x);

  if (x->optimize) vp8_optimize_mbuv(x);
}