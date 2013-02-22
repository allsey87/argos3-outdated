#include "qtopengl_lua_statetree_model.h"
#include "qtopengl_lua_statetree_item.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/wrappers/lua/lua_utility.h>

namespace argos {

   /****************************************/
   /****************************************/

   CQTOpenGLLuaStateTreeModel::CQTOpenGLLuaStateTreeModel(lua_State* pt_state,
                                                          QObject* pc_parent) :
      QAbstractItemModel(pc_parent),
      m_ptState(pt_state) {
      m_pcDataRoot = new CQTOpenGLLuaStateTreeItem();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLLuaStateTreeModel::~CQTOpenGLLuaStateTreeModel() {
      delete m_pcDataRoot;
   }

   /****************************************/
   /****************************************/

   QVariant CQTOpenGLLuaStateTreeModel::data(const QModelIndex& c_index,
                                             int n_role) const {
      if(!c_index.isValid()) {
         return QVariant();
      }
      if(n_role != Qt::DisplayRole) {
         return QVariant();
      }
      CQTOpenGLLuaStateTreeItem* pcItem = static_cast<CQTOpenGLLuaStateTreeItem*>(c_index.internalPointer());
      return pcItem->GetData(c_index.column());
   }

   /****************************************/
   /****************************************/

   Qt::ItemFlags CQTOpenGLLuaStateTreeModel::flags(const QModelIndex& c_index) const {
      if (!c_index.isValid()) {
         return 0;
      }
      else {
         return Qt::ItemIsEnabled;
      }
   }

   /****************************************/
   /****************************************/

   QModelIndex CQTOpenGLLuaStateTreeModel::index(int n_row,
                                                 int n_column,
                                                 const QModelIndex& c_parent) const {
      if(!hasIndex(n_row, n_column, c_parent)) {
         return QModelIndex();
      }
      CQTOpenGLLuaStateTreeItem* pcParentItem;
      if(!c_parent.isValid()) {
         pcParentItem = m_pcDataRoot;
      }
      else {
         pcParentItem = static_cast<CQTOpenGLLuaStateTreeItem*>(c_parent.internalPointer());
      }
      CQTOpenGLLuaStateTreeItem* pcChildItem = pcParentItem->GetChild(n_row);
      if(pcChildItem) {
         return createIndex(n_row, n_column, pcChildItem);
      }
      else {
         return QModelIndex();
      }
   }

   /****************************************/
   /****************************************/

   QModelIndex CQTOpenGLLuaStateTreeModel::parent(const QModelIndex& c_index) const {
      if (!c_index.isValid()) {
         return QModelIndex();
      }
      CQTOpenGLLuaStateTreeItem* pcChildItem = static_cast<CQTOpenGLLuaStateTreeItem*>(c_index.internalPointer());
      CQTOpenGLLuaStateTreeItem* pcParentItem = pcChildItem->GetParent();
      if (pcParentItem == m_pcDataRoot) {
         return QModelIndex();
      }
      else {
         return createIndex(pcParentItem->GetRow(), 0, pcParentItem);
      }
   }

   /****************************************/
   /****************************************/

   int CQTOpenGLLuaStateTreeModel::rowCount(const QModelIndex& c_parent) const {
      CQTOpenGLLuaStateTreeItem* pcParentItem;
      if(c_parent.column() > 0) {
         return 0;
      }
      if(!c_parent.isValid()) {
         pcParentItem = m_pcDataRoot;
      }
      else {
         pcParentItem = static_cast<CQTOpenGLLuaStateTreeItem*>(c_parent.internalPointer());
      }
      return pcParentItem->GetNumChildren();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLuaStateTreeModel::SetLuaState(lua_State* pt_state) {
      m_ptState = pt_state;
      Refresh(0);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLuaStateTreeModel::Refresh(int) {
      beginResetModel();
      delete m_pcDataRoot;
      m_pcDataRoot = new CQTOpenGLLuaStateTreeItem();
      lua_pushnil(m_ptState);
      lua_getglobal(m_ptState, "_G");
      ProcessLuaState(m_ptState, m_pcDataRoot);
      m_pcDataRoot->SortChildren();
      lua_pop(m_ptState, 2);
      endResetModel();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLuaStateTreeModel::ProcessLuaState(lua_State* pt_state,
                                                    CQTOpenGLLuaStateTreeItem* pc_item) {
      QList<QVariant> cData;
      switch(lua_type(pt_state, -2)) {
         case LUA_TBOOLEAN:
            cData << lua_toboolean(pt_state, -2);
            break;
         case LUA_TNUMBER:
            cData << lua_tonumber(pt_state, -2);
            break;
         case LUA_TSTRING:
            cData << lua_tostring(pt_state, -2);
            break;
         default: break;
      }
      if(lua_istable(pt_state, -1)) {
         CQTOpenGLLuaStateTreeItem* pcChild = new CQTOpenGLLuaStateTreeItem(cData, pc_item);
         pc_item->AddChild(pcChild);
         lua_pushnil(pt_state);
         while(lua_next(pt_state, -2)) {
            if(IsTypeVisitable(pt_state)) {
               ProcessLuaState(pt_state, pcChild);
            }
            lua_pop(pt_state, 1);
         }
         if(pcChild->GetNumChildren() == 0) {
            pc_item->RemoveChild(pcChild);
         }
      }
      else {
         switch(lua_type(pt_state, -1)) {
            case LUA_TBOOLEAN:
               cData << lua_toboolean(pt_state, -1);
               pc_item->AddChild(new CQTOpenGLLuaStateTreeItem(cData, pc_item));
               break;
            case LUA_TNUMBER:
               cData << lua_tonumber(pt_state, -1);
               pc_item->AddChild(new CQTOpenGLLuaStateTreeItem(cData, pc_item));
               break;
            case LUA_TSTRING:
               cData << lua_tostring(pt_state, -1);
               pc_item->AddChild(new CQTOpenGLLuaStateTreeItem(cData, pc_item));
               break;
            case LUA_TFUNCTION:
               cData[0] = cData[0].toString() + tr("()");
               pc_item->AddChild(new CQTOpenGLLuaStateTreeItem(cData, pc_item));
               break;
            default:
               break;
         }
      }
   }

   /****************************************/
   /****************************************/
   
   CQTOpenGLLuaStateTreeVariableModel::CQTOpenGLLuaStateTreeVariableModel(lua_State* pt_state,
                                                                          QObject* pc_parent) :
      CQTOpenGLLuaStateTreeModel(pt_state, pc_parent) {}

   /****************************************/
   /****************************************/

   QVariant CQTOpenGLLuaStateTreeVariableModel::headerData(int n_section,
                                                           Qt::Orientation e_orientation,
                                                           int n_role) const {
      if(e_orientation != Qt::Horizontal ||
         n_role != Qt::DisplayRole ||
         n_section > 1) {
         return QVariant();
      }
      else {
         return n_section == 0 ? tr("Variable") : tr("Value");
      }
   }

   /****************************************/
   /****************************************/

   int CQTOpenGLLuaStateTreeVariableModel::columnCount(const QModelIndex&) const {
      return 2;
   }

   /****************************************/
   /****************************************/

   bool CQTOpenGLLuaStateTreeVariableModel::IsTypeVisitable(lua_State* pt_state) {
      int nValueType = lua_type(pt_state, -1);
      int nKeyType = lua_type(pt_state, -2);
      if(nValueType == LUA_TSTRING || nValueType == LUA_TNUMBER) {
         if(nKeyType == LUA_TNUMBER) {
            return true;
         }
         else if(nKeyType == LUA_TSTRING) {
            return std::string(lua_tostring(pt_state, -2)) != "_VERSION";
         }
      }
      else if(nValueType == LUA_TTABLE) {
         if(nKeyType == LUA_TNUMBER) {
            return true;
         }
         else if(nKeyType == LUA_TSTRING) {
            return 
               std::string(lua_tostring(pt_state, -2)) != "_G" &&
               std::string(lua_tostring(pt_state, -2)) != "package" &&
               std::string(lua_tostring(pt_state, -2)) != "os" &&
               std::string(lua_tostring(pt_state, -2)) != "debug" &&
               std::string(lua_tostring(pt_state, -2)) != "coroutine";
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/
   
   CQTOpenGLLuaStateTreeFunctionModel::CQTOpenGLLuaStateTreeFunctionModel(lua_State* pt_state,
                                                                          QObject* pc_parent) :
      CQTOpenGLLuaStateTreeModel(pt_state, pc_parent) {}

   /****************************************/
   /****************************************/

   QVariant CQTOpenGLLuaStateTreeFunctionModel::headerData(int n_section,
                                                           Qt::Orientation e_orientation,
                                                           int n_role) const {
      return QVariant();
   }

   /****************************************/
   /****************************************/

   int CQTOpenGLLuaStateTreeFunctionModel::columnCount(const QModelIndex&) const {
      return 1;
   }

   /****************************************/
   /****************************************/

   bool CQTOpenGLLuaStateTreeFunctionModel::IsTypeVisitable(lua_State* pt_state) {
      int nValueType = lua_type(pt_state, -1);
      int nKeyType = lua_type(pt_state, -2);
      if(nValueType == LUA_TFUNCTION) {
         return true;
      }
      else if(nValueType == LUA_TTABLE) {
         if(nKeyType == LUA_TNUMBER) {
            return true;
         }
         else if(nKeyType == LUA_TSTRING) {
            return 
               std::string(lua_tostring(pt_state, -2)) != "_G" &&
               std::string(lua_tostring(pt_state, -2)) != "package" &&
               std::string(lua_tostring(pt_state, -2)) != "os" &&
               std::string(lua_tostring(pt_state, -2)) != "debug" &&
               std::string(lua_tostring(pt_state, -2)) != "coroutine";
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/
   
}
