<%@ include file="header.jsp" %>
<%@ include file="sidebar.jsp" %>
 <section id="main-content">
   <section class="wrapper">
      <div class="row">
                  <div class="col-lg-6">
                      <section class="panel">
                          <header class="panel-heading">
                             <strong>Add Category </strong>                 </header>
                          <div class="panel-body">
                          <font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
                              <form role="form" action="<%=request.getContextPath() %>/Admin" method="post">
                              <input type="hidden" name="action" value="create-user1" >
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Name <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="name"  type="text" required />
                                          </div>
                                      </div>
                                   <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Email <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="email"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Password <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="password"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">City <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="city"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Contact <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="contact"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Priviledge <span class="required">*</span></label>
                                          <select name="priviledge" class="form-control  m-bot13">
											<option value="1"> News Creator[Journalist]</option>
											<option value="2"> Ad Executive</option>
											</select>
                                      </div>
                                      
                                  <div class="form-group">
                                          <div class="col-lg-offset-2 col-lg-10">
                                              <button class="btn btn-primary" type="submit">Create User</button>
                                              
                                          </div>
                                      </div>
                                 </form>
                                  </div>
                   
                      </section>
                  </div>
                  </div>
                  </section>
                  </section>
                  <%@ include file="footer.jsp" %>