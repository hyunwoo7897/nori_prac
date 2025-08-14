#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h> 
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>


NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList& props){

    }


    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Intersection its_x, its_y;
        if (!scene->rayIntersect(ray, its_x)) {
            return Color3f(0.f);
        }

        Color3f emitted = Color3f(0.0f);
        Color3f reflected = Color3f(0.0f);

        if (its_x.mesh->getBSDF()->isDiffuse()) {
            Point2f sample = sampler->next2D();
            Point3f samplePos;
            Normal3f sampleNormal;
            if (its_x.mesh->isEmitter()) {
                emitted = its_x.mesh->getEmitter()->sample(sample, samplePos, sampleNormal);
            }
            if (scene->getEmitters().size() == 0) {
                return Color3f(0.0f); // Emitter가 없으면 바로 반환
            }
            uint32_t emitterIdx = floor(sampler->next1D() * scene->getEmitters().size());
            Emitter *emitter = scene->getEmitters()[emitterIdx];


            
            Color3f lightEmmited = emitter->sample(sample, samplePos, sampleNormal); // L_e에 이미 light쪽 pdf적용 완료
            if (sampleNormal.dot(its_x.p - samplePos) < 0){
                reflected = Color3f(0.0f);
            }
            else{
                Vector3f shadowDir = samplePos - its_x.p;
                float shadowDist = shadowDir.squaredNorm();
                shadowDir.normalize();

                
                Ray3f shadowRay(its_x.p, shadowDir);
                shadowRay.mint = Epsilon;
                shadowRay.maxt = std::sqrt(shadowDist) - Epsilon; 

                if( scene->rayIntersect(shadowRay, its_y) ) { 
                    reflected = Color3f(0.0f); // 그림자에 가려진 경우
                } else {
                    BSDFQueryRecord bRec( its_x.toLocal(-ray.d),its_x.toLocal(samplePos - its_x.p).normalized(),  ESolidAngle, &its_x);
                    //Color3f bsdfValue = its_x.mesh->getBSDF()->sample(bRec, sample);
                    Color3f bsdfValue = its_x.mesh->getBSDF()->eval(bRec);
                    float geometry = std::abs(its_x.shFrame.cosTheta(its_x.toLocal(samplePos - its_x.p).normalized()) * sampleNormal.dot((its_x.p - samplePos).normalized())) / (its_x.p - samplePos).squaredNorm();
                    reflected = geometry * bsdfValue.cwiseProduct(lightEmmited) * scene->getEmitters().size();
                }
            }
        
        }
        else if (its_x.mesh->getBSDF()->isSpecular()) {
            
            Point2f sample = sampler->next2D();
            BSDFQueryRecord bRec(its_x.toLocal(-ray.d), its_x.toLocal(Normal3f(0.0f, 0.0f, 1.0f)), ESolidAngle, &its_x);

            if (sample.x() < 0.95f) {
                Color3f bsdfValue = its_x.mesh->getBSDF()->sample(bRec,sample);
                Ray3f reflectedRay(its_x.p, its_x.toWorld(bRec.wo));
                reflectedRay.mint = Epsilon;
                reflectedRay.maxt = std::numeric_limits<float>::infinity();
                reflected = bsdfValue * this->Li(scene, sampler, reflectedRay) / 0.95f;   
            }
            else {
                reflected = Color3f(0.0f);
            }
        }

        return emitted + reflected;
    }

    std::string toString() const override { return "WhittedIntegrator[]"; }


protected:
    
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");




NORI_NAMESPACE_END