#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h> 
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>


NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList& props){

    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {

        Color3f total(0.0f);
        Color3f throughput(1.0f);
        Ray3f curRay = ray;
        bool isSpecularRay = true;
        float eta = 1.0f;
        float prob = 1.0f;

        int maxDepth = 50;
        for (int i = 0; i < maxDepth; i++) {
            Color3f emitted(0.0f);
            Color3f reflected(0.0f);
            Intersection its;
            Point2f sample = sampler->next2D();
            Point3f samplePos;
            Normal3f sampleNormal;
            if (!scene->rayIntersect(curRay, its)) {
                break;
            }
                
            // Direct lighting sampling
            if(scene->getEmitters().size() != 0){
                uint32_t emitterIdx = floor(sampler->next1D() * scene->getEmitters().size());
                Emitter *emitter = scene->getEmitters()[emitterIdx];
                Color3f lightEmitted = emitter->sample(sample, samplePos, sampleNormal);

                Vector3f shadowDir = samplePos - its.p;
                float shadowDist = shadowDir.squaredNorm();
                shadowDir.normalize();
                Ray3f shadowRay(its.p, shadowDir);
                shadowRay.mint = Epsilon;
                shadowRay.maxt = std::sqrt(shadowDist) - Epsilon;

                if (sampleNormal.dot(its.p - samplePos) > 0 && !scene->rayIntersect(shadowRay) &&! (Frame::cosTheta(its.toLocal(samplePos - its.p)) <= 0)) {
                    
                    BSDFQueryRecord bRec(its.toLocal(-curRay.d), its.toLocal(samplePos - its.p).normalized(), ESolidAngle, &its);
                    Color3f bsdfValue = its.mesh->getBSDF()->eval(bRec);
                    float geometry = std::abs(its.shFrame.cosTheta(its.toLocal(samplePos - its.p).normalized()) * sampleNormal.dot((its.p - samplePos).normalized())) / (its.p - samplePos).squaredNorm();
                    
                    float pLight = emitter->pdf(bRec) * shadowDist / (sampleNormal.normalized().dot((its.p - samplePos).normalized())) / scene->getEmitters().size();
                    float pBrdf = its.mesh->getBSDF()->pdf(bRec);
                    float wLight = 0.0f;
                    if((pLight + pBrdf) != 0 && pLight >=0 && pBrdf >=0){
                        wLight = pLight / ((pLight + pBrdf));
                    } 
                    
                    
                    total += (throughput * geometry * bsdfValue.cwiseProduct(lightEmitted) * scene->getEmitters().size() / prob) * wLight  ;
                    
                }
                
            }
            // Emission
            if (its.mesh->isEmitter() && isSpecularRay) {
                emitted =  its.mesh->getEmitter()->eval(its.toLocal(-curRay.d.normalized()),&its);
                total += (throughput * emitted / prob);
            }

            // Indirect lighting sampling

            sample = sampler->next2D();
            BSDFQueryRecord bRec(its.toLocal(-curRay.d),  its.toLocal(Normal3f(0.0f, 0.0f, 1.0f)), ESolidAngle, &its);
            Color3f bsdfValue = its.mesh->getBSDF()->sample(bRec, sample);
            isSpecularRay = !its.mesh->getBSDF()->isDiffuse();
            
            Intersection nextIts;
            
            if (scene->rayIntersect(Ray3f(its.p, its.toWorld(bRec.wo)), nextIts)){
                if(nextIts.mesh->isEmitter()){
                    Vector3f distance = its.p - nextIts.p;
                    Color3f emitted = nextIts.mesh->getEmitter()->eval(nextIts.toLocal(-its.toWorld(bRec.wo)));
                
                    float pLight = nextIts.mesh->getEmitter()->pdf(bRec) 
                                * distance.squaredNorm() 
                                / Frame::cosTheta(nextIts.toLocal((its.p - nextIts.p).normalized())) 
                                / scene->getEmitters().size();

                    float pBrdf = its.mesh->getBSDF()->pdf(bRec);
                    float wBrdf = 0.0f;
                    if((pLight + pBrdf) != 0 && pLight >=0 && pBrdf >=0){
                        wBrdf = pBrdf / ((pLight + pBrdf));
                    } 
                    total += throughput * emitted * bsdfValue * wBrdf / prob;
                    
                
                }
            } else{
                break;
            }

            throughput *= bsdfValue / prob;
            

            curRay = Ray3f(its.p, its.toWorld(bRec.wo));
            curRay.mint = Epsilon;
            curRay.maxt = std::numeric_limits<float>::infinity();

            // Russian roulette
            eta *= bRec.eta;
            prob = i < 3 ? 1.0f : std::min(throughput.maxCoeff() *eta*eta, 0.99f);
            if(sampler->next1D() > prob)
                break;
        }
        return total;
    }
    


    std::string toString() const override { return "PathMisIntegrator[]"; }


protected:
    
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");




NORI_NAMESPACE_END