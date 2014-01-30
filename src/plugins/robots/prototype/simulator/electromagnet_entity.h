/**
 * @file <argos3/plugins/robot/prototype/simulator/electromagnet_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef ELECTROMAGNET_ENTITY_H
#define ELECTROMAGNET_ENTITY_H

namespace argos {
   class CElectromagnetEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   class CElectromagnetEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CElectromagnetEntity*> TList;

   public:

      CElectromagnetEntity(CComposableEntity* pc_parent);

      CElectromagnetEntity(CComposableEntity* pc_parent,
                           const std::string& str_id,
                           const CVector3& c_passive_field,
                           const CVector3& c_active_field);

      virtual ~CElectromagnetEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      /**
       * Returns the current field of the electromagnet.
       * @return the current field of the electromagnet.
       * @see SetField()
       */
      inline const CVector3& GetField() const {
         return m_cField;
      }

      /**
       * Returns the passive field with which the electromagnet was initialized.
       * When the simulation is reset, the electromagnet field is set to this value.
       * @return the field with which the electromagnet was initialized.
       * @see GetField()
       */
      inline const CVector3& GetPassiveField() const {
         return m_cPassiveField;
      }

      /**
       * Returns the active field with which the electromagnet was initialized.
       * This value is used by an actuator, that will update the field using the
       * expression: field = passive_field + active_field * current
       * @return the active field with which the electromagnet was initialized.
       * @see GetField()
       */
      inline const CVector3& GetActiveField() const {
         return m_cActiveField;
      }


      /**
       * Sets the current field of the electromagnet.
       * @param c_field the wanted field.
       * @see GetField()
       */
      inline void SetField(const CVector3& c_field) {
         m_cField = c_field;
      }


      virtual std::string GetTypeDescription() const {
         return "electromagnet";
      }

   protected:

      CVector3 m_cField;
      CVector3 m_cPassiveField;
      CVector3 m_cActiveField;

   };

   /****************************************/
   /****************************************/

}

#endif
