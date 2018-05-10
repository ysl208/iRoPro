#ifndef _RAPID_PBD_PROGRAM_DB_H_
#define _RAPID_PBD_PROGRAM_DB_H_

#include <map>
#include <string>

#include "mongodb_store/message_store.h"
#include "rapid_pbd_msgs/Program.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_pbd/db_names.h"

namespace rapid {
namespace pbd {
static const char kProgramListTopic[] = "program_list";
static const char kDomainListTopic[] = "domain_list";

class SceneDb {
 public:
  explicit SceneDb(mongodb_store::MessageStoreProxy* db);

  std::string Insert(const sensor_msgs::PointCloud2& cloud);
  bool Get(const std::string& db_id, sensor_msgs::PointCloud2* cloud) const;
  bool Delete(const std::string& db_id);

 private:
  mongodb_store::MessageStoreProxy* db_;
};

class ProgramDb {
 public:
  ProgramDb(const ros::NodeHandle& nh, mongodb_store::MessageStoreProxy* db,
            ros::Publisher* list_pub);

  // Publishes the first message.
  void Start();

  std::string Insert(const rapid_pbd_msgs::Program& program);
  void Update(const std::string& db_id, const rapid_pbd_msgs::Program& program);
  void StartPublishingProgramById(const std::string& db_id);
  bool Get(const std::string& db_id, rapid_pbd_msgs::Program* program) const;
  bool GetByName(const std::string& name,
                 rapid_pbd_msgs::Program* program) const;
  void Delete(const std::string& db_id);
  bool GetList(std::vector<std::string>* names);

 private:
  ros::NodeHandle nh_;
  mongodb_store::MessageStoreProxy* db_;
  ros::Publisher* list_pub_;
  std::map<std::string, ros::Publisher> program_pubs_;

  void PublishList();
  void PublishProgram(const std::string& db_id);
};

class DomainDb {
 public:
  DomainDb(const ros::NodeHandle& nh, mongodb_store::MessageStoreProxy* domain,
            ros::Publisher* list_pub);

  // Publishes the first message.
  void Start();

  std::string Insert(const rapid_pbd_msgs::Domain& domain);
  void Update(const std::string& domain_id, const rapid_pbd_msgs::Domain& domain);
  void StartPublishingDomainById(const std::string& domain_id);
  bool Get(const std::string& domain_id, rapid_pbd_msgs::Domain* domain) const;
  bool GetByName(const std::string& name,
                 rapid_pbd_msgs::Domain* domain) const;
  void Delete(const std::string& domain_id);
  bool GetList(std::vector<std::string>* names);

 private:
  ros::NodeHandle nh_;
  mongodb_store::MessageStoreProxy* domain_;
  ros::Publisher* list_pub_;
  std::map<std::string, ros::Publisher> domain_pubs_;

  void PublishList();
  void PublishDomain(const std::string& domain_id);
};


DomainDb::DomainDb(const ros::NodeHandle& nh,
                     mongodb_store::MessageStoreProxy* db,
                     ros::Publisher* list_pub)
    : nh_(nh), domain_(db), list_pub_(list_pub), domain_pubs_() {}

void DomainDb::Start() { PublishList(); }

std::string DomainDb::Insert(const rapid_pbd_msgs::Domain& domain) {
  std::string id = domain_->insert(domain);
  PublishList();
  return id;
}

void DomainDb::Update(const std::string& domain_id,
                       const rapid_pbd_msgs::Domain& domain) {
  bool success = domain_->updateID(domain_id, domain);
  if (!success) {
    ROS_ERROR("Failed to update domain with ID: \"%s\"", domain_id.c_str());
    return;
  }
  PublishList();
  PublishDomain(domain_id);
}

void DomainDb::StartPublishingDomainById(const std::string& domain_id) {
  if (domain_pubs_.find(domain_id) != domain_pubs_.end()) {
    return;
  }
  vector<shared_ptr<Domain> > results;
  bool success = domain_->queryID(domain_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Can't start publishing domain with ID: \"%s\"", domain_id.c_str());
    return;
  }
  ros::Publisher pub = nh_.advertise<Domain>("domain/" + domain_id, 1, true);
  domain_pubs_[domain_id] = pub;
  domain_pubs_[domain_id].publish(results[0]);
}

bool DomainDb::Get(const std::string& domain_id,
                    rapid_pbd_msgs::Domain* domain) const {
  vector<shared_ptr<Domain> > results;
  bool success = domain_->queryID(domain_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Can't get domain with ID: \"%s\"", domain_id.c_str());
    return false;
  }
  *domain = *results[0];
  return true;
}

// ******
// Planning domain database
bool DomainDb::GetByName(const std::string& name,
                          rapid_pbd_msgs::Domain* domain) const {
  vector<shared_ptr<Domain> > results;
  mongo::BSONObj query = BSON("name" << name);
  mongo::BSONObj meta_query;
  mongo::BSONObj sort_query;
  bool find_one = true;
  bool decode_metas = false;
  int limit = 1;

  vector<std::pair<shared_ptr<Domain>, mongo::BSONObj> > msg_and_metas;
  bool success = domain_->query(msg_and_metas, query, meta_query, sort_query,
                            find_one, decode_metas, limit);
  if (!success || msg_and_metas.size() < 1) {
    ROS_ERROR("Can't get domain with name: \"%s\"", name.c_str());
    return false;
  }
  shared_ptr<Domain> domain_p = msg_and_metas[0].first;
  if (!domain_p) {
    ROS_ERROR("Database returned null message for name: \"%s\"", name.c_str());
    return false;
  }
  *domain = *domain_p;
  return true;
}

void DomainDb::Delete(const std::string& domain_id) {
  bool success = domain_->deleteID(domain_id);

  if (success) {
    PublishList();
    if (domain_pubs_.find(domain_id) != domain_pubs_.end()) {
      domain_pubs_[domain_id].shutdown();
      domain_pubs_.erase(domain_id);
    }
  } else {
    ROS_ERROR("Could not delete domain with ID \"%s\"", domain_id.c_str());
  }
}

void DomainDb::PublishList() {
  if (list_pub_ == NULL) {
    return;
  }
  vector<pair<shared_ptr<Domain>, mongo::BSONObj> > results;
  domain_->query<Domain>(results);
  DomainInfoList msg;
  for (size_t i = 0; i < results.size(); ++i) {
    DomainInfo info;
    info.name = results[i].first->name;
    info.domain_id = results[i].second.getField("_id").OID().toString();
    msg.domains.push_back(info);
  }
  list_pub_->publish(msg);
}

bool DomainDb::GetList(std::vector<std::string>* names) {
  vector<pair<shared_ptr<Domain>, mongo::BSONObj> > results;
  domain_->query<Domain>(results);
  for (size_t i = 0; i < results.size(); ++i) {
    std::string name;
    name = results[i].first->name;
    names->push_back(name);
  }
}

void DomainDb::PublishDomain(const std::string& domain_id) {
  if (domain_pubs_.find(domain_id) == domain_pubs_.end()) {
    ROS_ERROR("No publisher for domain ID: \"%s\"", domain_id.c_str());
    return;
  }
  vector<shared_ptr<Domain> > results;
  bool success = domain_->queryID(domain_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Could not republish domain with ID: \"%s\"", domain_id.c_str());
    return;
  }
  domain_pubs_[domain_id].publish(results[0]);
}

}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PROGRAM_DB_H_
