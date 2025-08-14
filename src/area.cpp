#include <nori/common.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>


NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter{
    public: 
        AreaLight(const PropertyList &plist) {
            m_radiance = plist.getColor("radiance");
         }

        bool isConnectMesh() const override {
            return true; // AreaLight는 Mesh에 연결되어야 함
        }
        void setConnectMesh(Mesh *mesh) override {
            this->m_mesh = mesh;
        }

        Color3f sample(const Point2f &sample, Point3f &samplePos, Normal3f &sampleNormal) const {

            // AreaLight는 반드시 Mesh에 붙어 있어야 함
            if (!m_mesh)
                throw NoriException("AreaLight: No mesh attached!");
            // Mesh의 samplePosition을 그대로 사용

            float pdf = m_mesh->samplePosition(sample, samplePos, sampleNormal);

            return m_radiance / pdf; // radiance를 pdf로 나누어 반환
        }

        Color3f eval(const Vector3f &wo, Intersection *its = nullptr) const override {
            // AreaLight는 항상 radiance를 반환
            return m_radiance;
        }

        /// Compute the density of \ref sample() wrt. solid angles
        float pdf(const BSDFQueryRecord &bRec) const {
            return 1 / m_mesh->getSurfaceArea(); // AreaLight의 pdf는 면적에 반비례
        }

        std::string toString() const override {
            return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                //"  mesh = %s\n"
                "]",
                m_radiance.toString()
                //m_mesh ? indent(m_mesh->toString()) : std::string("null")
            );
        }

    private:
        
};


NORI_REGISTER_CLASS(AreaLight, "area");

NORI_NAMESPACE_END

