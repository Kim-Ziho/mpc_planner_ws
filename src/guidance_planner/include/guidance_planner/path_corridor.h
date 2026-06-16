#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/node.h>
#include <guidance_planner/types/paths.h>

#include <Eigen/Dense>

#include <vector>

namespace GuidancePlanner
{

  /**
   * @brief STRRT 샘플링 가이드 전용 시공간 corridor.
   *
   * PRM best GeometricPath 의 (x,y,k) 폴리라인을 호길이 파라미터화한 경량 표현.
   * cubic/tk::spline 보간 없이 세그먼트 선형보간 + 닫힌 식으로 다음 3개만 제공한다:
   *   - point(s)  : 호길이 s 의 위치
   *   - normal(s) : 그 지점 접선의 좌수직 단위벡터
   *   - time(s)   : 그 지점의 시각 [s]
   *
   * @note 생성 시 GeometricPath 의 노드 위치/시각을 값으로 복사하므로, 원본 PRM
   *       그래프 노드가 이후 무효화되어도 안전하다 (Node* 를 보관하지 않음).
   */
  struct PathCorridor
  {
    PathCorridor() = default;

    explicit PathCorridor(const GeometricPath &path)
    {
      const std::vector<Node *> nodes = path.GetNodes();
      pts_.reserve(nodes.size());
      t_.reserve(nodes.size());
      for (const Node *n : nodes)
      {
        if (n == nullptr)
          continue;
        pts_.emplace_back(n->point_.Pos());     // (x, y)
        t_.push_back(n->point_.Time() * Config::DT); // k → seconds
      }

      // 누적 호길이
      s_cum_.assign(pts_.size(), 0.0);
      for (size_t i = 1; i < pts_.size(); ++i)
        s_cum_[i] = s_cum_[i - 1] + (pts_[i] - pts_[i - 1]).norm();

      valid_ = pts_.size() >= 2 && s_cum_.back() > 1e-6;
    }

    bool valid() const { return valid_; }
    double length() const { return valid_ ? s_cum_.back() : 0.0; }

    /** @brief 호길이 s∈[0,length] 위치 (세그먼트 선형보간, 범위 밖은 끝점 clamp) */
    Eigen::Vector2d point(double s) const
    {
      size_t i; double a;
      locate(s, i, a);
      return (1.0 - a) * pts_[i] + a * pts_[i + 1];
    }

    /** @brief 호길이 s 의 좌수직 단위벡터 (접선 t=(tx,ty) → n=(-ty,tx)) */
    Eigen::Vector2d normal(double s) const
    {
      size_t i; double a;
      locate(s, i, a);
      Eigen::Vector2d tangent = pts_[i + 1] - pts_[i];
      double len = tangent.norm();
      if (len < 1e-9)
        return Eigen::Vector2d(0.0, 1.0);
      tangent /= len;
      return Eigen::Vector2d(-tangent.y(), tangent.x());
    }

    /** @brief 호길이 s 의 시각 [s] (세그먼트 선형보간) */
    double time(double s) const
    {
      size_t i; double a;
      locate(s, i, a);
      return (1.0 - a) * t_[i] + a * t_[i + 1];
    }

  private:
    /** @brief s 가 속한 세그먼트 인덱스 i 와 보간계수 a∈[0,1] (s = pts_[i] + a·(pts_[i+1]-pts_[i])) */
    void locate(double s, size_t &i, double &a) const
    {
      const double L = s_cum_.back();
      s = std::min(std::max(s, 0.0), L);
      // 마지막 세그먼트로 자연스럽게 떨어지도록 상한 처리
      i = 0;
      while (i + 2 < pts_.size() && s_cum_[i + 1] <= s)
        ++i;
      const double seg = s_cum_[i + 1] - s_cum_[i];
      a = seg > 1e-9 ? (s - s_cum_[i]) / seg : 0.0;
    }

    std::vector<Eigen::Vector2d> pts_;
    std::vector<double> t_;     // 노드 시각 [s]
    std::vector<double> s_cum_; // 누적 호길이
    bool valid_{false};
  };

} // namespace GuidancePlanner
