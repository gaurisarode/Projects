<%@ include file="header.jsp" %>
<%@ include file="sidebar.jsp" %>
 
 <sql:query dataSource="${ds}" var="result">
SELECT * from users where id = <c:out value="${id }"></c:out>;
</sql:query> 

  <section id="main-content">
   <section class="wrapper">
      <div class="row">
                  <div class="col-lg-6">
                      <section class="panel">
                          <header class="panel-heading">
                             <strong>Edit User Details</strong>                 </header>
                          <div class="panel-body">
                          <font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
                              <form role="form" action="<%=request.getContextPath() %>/Admin" method="post">
                              <c:forEach var="row" items="${result.rows}">
                              <input type="hidden" name="action" value="user-edit1" >
								<input type="hidden" name="id" value="<c:out value="${row.id}"/>" >
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Name <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="name" value="<c:out value="${row.name}"/>" type="text" required />
                                          </div>
                                      </div>
                                   <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Email <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="email" value="<c:out value="${row.email}"/>" type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Password <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="password" value="<c:out value="${row.password}"/>" type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">City <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="city" value="<c:out value="${row.city}"/>" type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Contact <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="contact" value="<c:out value="${row.contact}"/>" type="text" required />
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
                                              <button class="btn btn-primary" type="submit">Edit User Details</button>
                                              
                                          </div>
                                      </div>
                                      </c:forEach>
                                 </form>
                                  </div>
                   
                      </section>
                  </div>
                  </div>
                  </section>
                  </section>
                  <%@ include file="footer.jsp" %>