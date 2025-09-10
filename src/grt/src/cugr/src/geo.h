//
// Some class templates for geometry primitives (point, interval, box)
//

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "utl/Logger.h"

namespace grt {

using utl::format_as;

// Point template
class PointT
{
 public:
  PointT(int x = std::numeric_limits<int>::max(),
         int y = std::numeric_limits<int>::max())
      : x_(x), y_(y)
  {
  }
  bool IsValid() { return *this != PointT(); }

  int x() const { return x_; }
  int y() const { return y_; }

  // Operators
  const int& operator[](const unsigned d) const
  {
    assert(d == 0 || d == 1);
    return (d == 0 ? x_ : y_);
  }
  int& operator[](const unsigned d)
  {
    assert(d == 0 || d == 1);
    return (d == 0 ? x_ : y_);
  }
  PointT operator+(const PointT& rhs)
  {
    return PointT(x_ + rhs.x_, y_ + rhs.y_);
  }
  PointT operator/(int divisor) { return PointT(x_ / divisor, y_ / divisor); }
  PointT& operator+=(const PointT& rhs)
  {
    x_ += rhs.x_;
    y_ += rhs.y_;
    return *this;
  }
  PointT& operator-=(const PointT& rhs)
  {
    x_ -= rhs.x_;
    y_ -= rhs.y_;
    return *this;
  }
  bool operator==(const PointT& rhs) const
  {
    return x_ == rhs.x_ && y_ == rhs.y_;
  }
  bool operator!=(const PointT& rhs) const { return !(*this == rhs); }

  friend std::ostream& operator<<(std::ostream& os, const PointT& pt)
  {
    os << "(" << pt.x_ << ", " << pt.y_ << ")";
    return os;
  }

 private:
  int x_;
  int y_;
};

// Interval template
class IntervalT
{
 public:
  int low, high;

  IntervalT() { Set(); }
  IntervalT(int val) { Set(val); }
  IntervalT(int lo, int hi) { Set(lo, hi); }

  // Setters
  void Set()
  {
    low = std::numeric_limits<int>::max();
    high = std::numeric_limits<int>::min();
  }
  void Set(int val)
  {
    low = val;
    high = val;
  }
  void Set(int lo, int hi)
  {
    low = lo;
    high = hi;
  }

  // Getters
  int center() const { return (high + low) / 2; }
  int range() const { return high - low; }

  // Update
  // Update() is always safe, FastUpdate() assumes existing values
  void Update(int newVal)
  {
    if (newVal < low) {
      low = newVal;
    }
    if (newVal > high) {
      high = newVal;
    }
  }
  void FastUpdate(int newVal)
  {
    if (newVal < low) {
      low = newVal;
    } else if (newVal > high) {
      high = newVal;
    }
  }

  // Two types of intervals: 1. normal, 2. degenerated (i.e., point)
  // is valid interval (i.e., valid closed interval)
  bool IsValid() const { return low <= high; }
  // is strictly valid interval (excluding degenerated ones, i.e., valid open
  // interval)
  bool IsStrictValid() const { return low < high; }

  // Geometric Query/Update
  // interval/range of union (not union of intervals)
  IntervalT UnionWith(const IntervalT& rhs) const
  {
    if (!IsValid()) {
      return rhs;
    }

    if (!rhs.IsValid()) {
      return *this;
    }

    return IntervalT(std::min(low, rhs.low), std::max(high, rhs.high));
  }
  // may return an invalid interval (as empty intersection)
  IntervalT IntersectWith(const IntervalT& rhs) const
  {
    return IntervalT(std::max(low, rhs.low), std::min(high, rhs.high));
  }
  bool HasIntersectWith(const IntervalT& rhs) const
  {
    return IntersectWith(rhs).IsValid();
  }
  bool HasStrictIntersectWith(const IntervalT& rhs) const
  {
    return IntersectWith(rhs).IsStrictValid();
  }
  // contain a val
  bool Contain(int val) const { return val >= low && val <= high; }
  bool StrictlyContain(int val) const { return val > low && val < high; }
  // get nearest point(s) to val (assume valid intervals)
  int GetNearestPointTo(int val) const
  {
    if (val <= low) {
      return low;
    }

    if (val >= high) {
      return high;
    }

    return val;
  }
  IntervalT GetNearestPointsTo(IntervalT val) const
  {
    if (val.high <= low) {
      return {low};
    }

    if (val.low >= high) {
      return {high};
    }

    return IntersectWith(val);
  }

  void ShiftBy(const int& rhs)
  {
    low += rhs;
    high += rhs;
  }

  // Operators
  bool operator==(const IntervalT& rhs) const
  {
    return (!IsValid() && !rhs.IsValid())
           || (low == rhs.low && high == rhs.high);
  }
  bool operator!=(const IntervalT& rhs) const { return !(*this == rhs); }

  friend std::ostream& operator<<(std::ostream& os, const IntervalT& interval)
  {
    os << "(" << interval.low << ", " << interval.high << ")";
    return os;
  }
};

// Box template
class BoxT
{
 public:
  IntervalT x, y;

  BoxT() { Set(); }
  BoxT(int xVal, int yVal) { Set(xVal, yVal); }
  BoxT(const PointT& pt) { Set(pt); }
  BoxT(int lx, int ly, int hx, int hy) { Set(lx, ly, hx, hy); }
  BoxT(const IntervalT& xRange, const IntervalT& yRange)
  {
    Set(xRange, yRange);
  }
  BoxT(const PointT& low, const PointT& high) { Set(low, high); }
  BoxT(const BoxT& box) { Set(box); }

  // Setters
  int& lx() { return x.low; }
  int& ly() { return y.low; }
  int& hy() { return y.high; }
  int& hx() { return x.high; }
  IntervalT& operator[](unsigned i)
  {
    assert(i == 0 || i == 1);
    return (i == 0) ? x : y;
  }
  void Set()
  {
    x.Set();
    y.Set();
  }
  void Set(int xVal, int yVal)
  {
    x.Set(xVal);
    y.Set(yVal);
  }
  void Set(const PointT& pt) { Set(pt.x(), pt.y()); }
  void Set(int lx, int ly, int hx, int hy)
  {
    x.Set(lx, hx);
    y.Set(ly, hy);
  }
  void Set(const IntervalT& xRange, const IntervalT& yRange)
  {
    x = xRange;
    y = yRange;
  }
  void Set(const PointT& low, const PointT& high)
  {
    Set(low.x(), low.y(), high.x(), high.y());
  }
  void Set(const BoxT& box) { Set(box.x, box.y); }

  // Two types of boxes: normal & degenerated (line or point)
  // is valid box
  bool IsValid() const { return x.IsValid() && y.IsValid(); }
  // is strictly valid box (excluding degenerated ones)
  bool IsStrictValid() const
  {
    return x.IsStrictValid() && y.IsStrictValid();
  }  // tighter

  // Getters
  int lx() const { return x.low; }
  int ly() const { return y.low; }
  int hy() const { return y.high; }
  int hx() const { return x.high; }
  int cx() const { return x.center(); }
  int cy() const { return y.center(); }
  int width() const { return x.range(); }
  int height() const { return y.range(); }
  int hp() const { return width() + height(); }  // half perimeter
  int area() const { return width() * height(); }
  const IntervalT& operator[](unsigned i) const
  {
    assert(i == 0 || i == 1);
    return (i == 0) ? x : y;
  }

  // Update() is always safe, FastUpdate() assumes existing values
  void Update(int xVal, int yVal)
  {
    x.Update(xVal);
    y.Update(yVal);
  }
  void FastUpdate(int xVal, int yVal)
  {
    x.FastUpdate(xVal);
    y.FastUpdate(yVal);
  }
  void Update(const PointT& pt) { Update(pt.x(), pt.y()); }
  void FastUpdate(const PointT& pt) { FastUpdate(pt.x(), pt.y()); }

  // Geometric Query/Update
  BoxT UnionWith(const BoxT& rhs) const
  {
    return {x.UnionWith(rhs.x), y.UnionWith(rhs.y)};
  }
  BoxT IntersectWith(const BoxT& rhs) const
  {
    return {x.IntersectWith(rhs.x), y.IntersectWith(rhs.y)};
  }
  bool HasIntersectWith(const BoxT& rhs) const
  {
    return IntersectWith(rhs).IsValid();
  }
  bool HasStrictIntersectWith(const BoxT& rhs) const
  {
    return IntersectWith(rhs).IsStrictValid();
  }  // tighter
  bool Contain(const PointT& pt) const
  {
    return x.Contain(pt.x()) && y.Contain(pt.y());
  }
  bool StrictlyContain(const PointT& pt) const
  {
    return x.StrictlyContain(pt.x()) && y.StrictlyContain(pt.y());
  }
  PointT GetNearestPointTo(const PointT& pt)
  {
    return {x.GetNearestPointTo(pt.x()), y.GetNearestPointTo(pt.y())};
  }
  BoxT GetNearestPointsTo(BoxT val) const
  {
    return {x.GetNearestPointsTo(val.x), y.GetNearestPointsTo(val.y)};
  }

  void ShiftBy(const PointT& rhs)
  {
    x.ShiftBy(rhs.x());
    y.ShiftBy(rhs.y());
  }

  bool operator==(const BoxT& rhs) const
  {
    return (x == rhs.x) && (y == rhs.y);
  }
  bool operator!=(const BoxT& rhs) const { return !(*this == rhs); }

  friend std::ostream& operator<<(std::ostream& os, const BoxT& box)
  {
    os << "[x: " << box.x << ", y: " << box.y << "]";
    return os;
  }
};

}  // namespace grt
