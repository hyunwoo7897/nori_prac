/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <nori/object.h>
#include <nori/mesh.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    bool isEmitter() const { return true; }
    EClassType getClassType() const { return EEmitter; }

    virtual bool isConnectMesh() const { return false; }
    virtual void setConnectMesh(Mesh * mesh) { return ;}
    
    virtual Color3f sample(const Point2f &sample, Point3f &samplePos, Normal3f &sampleNormal) const = 0;

    virtual Color3f eval(const Vector3f &wo, Intersection *its = nullptr) const = 0;
    
    virtual float pdf(const BSDFQueryRecord &bRec) const = 0;
    
    virtual std::string toString() const {
        return tfm::format(
            "Emitter[\n"
            "  radiance = %s,\n"
            
            "]",
            m_radiance.toString()
            //m_mesh ? indent(m_mesh->toString()) : std::string("null")
        );
    }

    Color3f getRadiance() const {
        return m_radiance;
    }

    void addChild(NoriObject *obj) {

    }
protected:
    Color3f m_radiance;
    Mesh *m_mesh;
};

NORI_NAMESPACE_END
