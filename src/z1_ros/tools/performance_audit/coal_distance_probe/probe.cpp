#include <coal/distance.h>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <memory>
#include <cstdlib>
#include <new>
static bool measuring=false; static unsigned allocations=0;
void* operator new(std::size_t n){if(measuring)++allocations;if(auto p=std::malloc(n))return p;throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p) noexcept{std::free(p);}
void operator delete[](void* p) noexcept{std::free(p);}
void operator delete(void* p,std::size_t) noexcept{std::free(p);}
void operator delete[](void* p,std::size_t) noexcept{std::free(p);}

using Vec = Eigen::Matrix<double, 6, 1>;
struct Pair { size_t a,b; std::unique_ptr<coal::ComputeDistance> compute; };
int main(int argc, char **argv) {
  pinocchio::Model model;
  pinocchio::GeometryModel geometry;
  pinocchio::urdf::buildModel(argv[1], model);
  pinocchio::urdf::buildGeom(model, argv[1], pinocchio::COLLISION, geometry);
  pinocchio::Data data(model);
  pinocchio::GeometryData gd(geometry);
  std::vector<Pair> pairs;
  for(size_t a=0; a<geometry.geometryObjects.size(); ++a)
    for(size_t b=a+1; b<geometry.geometryObjects.size(); ++b)
      if(geometry.geometryObjects[a].parentFrame != geometry.geometryObjects[b].parentFrame) {
        Pair p{a,b,{}};
        p.compute.reset(new coal::ComputeDistance(geometry.geometryObjects[a].geometry.get(),geometry.geometryObjects[b].geometry.get()));
        pairs.push_back(std::move(p));
      }
  auto query = [&](Pair &p, bool cached) {
    const auto &a = geometry.geometryObjects[p.a], &b = geometry.geometryObjects[p.b];
    const auto &at = gd.oMg[p.a], &bt = gd.oMg[p.b];
    coal::DistanceRequest fresh;
    fresh.enable_signed_distance = true;
    auto &request = fresh;
    coal::DistanceResult result;
    const double d=cached ? (*p.compute)(coal::Transform3s(at.rotation(),at.translation()),coal::Transform3s(bt.rotation(),bt.translation()),request,result) : coal::distance(a.geometry.get(),coal::Transform3s(at.rotation(),at.translation()),
                                 b.geometry.get(),coal::Transform3s(bt.rotation(),bt.translation()),
                                 request,result);
    
    return d;
  };
  std::mt19937 random(8675309);
  std::uniform_real_distribution<double> uniform(0,1);
  std::vector<Vec> random_q(300), smooth_q(2000);
  for(auto &q : random_q) for(int j=0;j<6;++j)
    q[j]=model.lowerPositionLimit[j] + uniform(random)*(model.upperPositionLimit[j]-model.lowerPositionLimit[j]);
  for(size_t i=0;i<smooth_q.size();++i) {
    auto &q=smooth_q[i]; q << 0,.8,-1.2,.4,0,0;
    for(int j=0;j<6;++j) q[j]+=.1*std::sin(i*.002*3.14159+j*.3);
  }
  auto test = [&](const char *label, const std::vector<Vec> &points) {
    double maximum_error=0, minimum=1e6; size_t differing=0;
    for(const auto &q:points) {
      pinocchio::updateGeometryPlacements(model,data,geometry,gd,q);
      for(auto &pair:pairs) {
        const double fresh=query(pair,false), warm=query(pair,true);
        const double error=std::abs(fresh-warm);
        minimum=std::min(minimum,fresh);
        maximum_error=std::max(maximum_error,error);
        differing += error>1e-8;
      }
    }
    auto bench=[&](bool cached) {
      volatile double sum=0; allocations=0; measuring=true;
      const auto start=std::chrono::steady_clock::now();
      for(int repeat=0;repeat<3;++repeat) for(const auto &q:points) {
        pinocchio::updateGeometryPlacements(model,data,geometry,gd,q);
        for(auto &pair:pairs) sum += query(pair,cached);
      }
      const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/(3*points.size()*pairs.size()); measuring=false;
      return std::pair<double,double>{us,double(allocations)/(3*points.size()*pairs.size())};
    };
    const auto fresh = bench(false), persistent = bench(true);
    std::cout<<label<<" pairs="<<pairs.size()<<" fresh_us="<<fresh.first<<" persistent_us="<<persistent.first<<" fresh_new="<<fresh.second<<" persistent_new="<<persistent.second
             <<" max_error="<<maximum_error<<" differences_over_1e-8="<<differing<<" min_distance="<<minimum<<'\n';
  };
  test("random",random_q); test("smooth",smooth_q);
}
